# dispatcher.py
"""
Phase 4 – Dispatcher
Connects the three subsystems:

    GamepadReader  →  AppState  →  SerialBridge

The dispatcher runs a single daemon thread that:
  1. Drains events from GamepadReader's queue.
  2. Normalises raw evdev axis values to [-1.0, 1.0].
  3. Applies a dead zone to analog sticks (values snap to 0.0 inside it).
  4. Tracks button state as a 2-byte bitmask (see BUTTON_BIT_MAP below),
     matching the layout expected by tx_firmware.ino / trailer_loader_rx.ino.
  5. Updates AppState via update_channel().
  6. Transmits the full channel + button snapshot over SerialBridge on
     *every* axis or button event — but only when a vehicle is selected.

Serial disconnections are detected on write failure and trigger a
blocking reconnect inside the thread (the queue keeps filling safely
in the meantime).

Usage::

    from gamepad      import GamepadReader
    from state        import AppState
    from serial_bridge import SerialBridge
    from dispatcher   import Dispatcher

    reader = GamepadReader()
    state  = AppState()
    bridge = SerialBridge()

    d = Dispatcher(reader, state, bridge)
    d.start()
    # … Flask app runs, vehicles get selected, etc. …
    d.stop()
"""

import logging
import queue
import threading
import time

from evdev import ecodes

log = logging.getLogger(__name__)

# ── Normalisation constants ───────────────────────────────────────────────────

# Analog sticks report ±32767; d-pad reports ±1.
STICK_MAX   = 32767.0
HAT_MAX     = 1.0

# Axis codes that use STICK_MAX scaling.
STICK_AXES  = frozenset({ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY})
HAT_AXES    = frozenset({ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y})

# Dead zone: raw stick values whose |value| ≤ this threshold are zeroed.
# Matches the flat value reported by the Switch controller (500).
DEAD_ZONE   = 500

# How long (seconds) to wait between serial reconnect attempts.
RECONNECT_DELAY = 2.0

# Queue drain timeout – keeps the thread responsive to stop().
QUEUE_TIMEOUT   = 0.1

# ── Button bit layout ──────────────────────────────────────────────────────
# Must match the ControlPayload.buttons[] layout in tx_firmware.ino /
# trailer_loader_rx.ino:
#
#   buttons[0]  bit0=A  bit1=B  bit2=X  bit3=Y  bit4=L1  bit5=R1  bit6=L2  bit7=R2
#   buttons[1]  bit0=ThumbL  bit1=ThumbR  (bits 2-7 reserved)
#
# Maps evdev BTN_* code → (byte index into the 2-byte bitmask, bit mask).
BUTTON_BIT_MAP = {
    ecodes.BTN_SOUTH:  (0, 1 << 0),  # A
    ecodes.BTN_EAST:   (0, 1 << 1),  # B
    ecodes.BTN_NORTH:  (0, 1 << 2),  # X
    ecodes.BTN_WEST:   (0, 1 << 3),  # Y
    ecodes.BTN_TL:     (0, 1 << 4),  # L1
    ecodes.BTN_TR:     (0, 1 << 5),  # R1
    ecodes.BTN_TL2:    (0, 1 << 6),  # L2
    ecodes.BTN_TR2:    (0, 1 << 7),  # R2
    ecodes.BTN_THUMBL: (1, 1 << 0),  # ThumbL
    ecodes.BTN_THUMBR: (1, 1 << 1),  # ThumbR
}


# ── Normalisation helpers ─────────────────────────────────────────────────────

def _normalise(code: int, raw: int) -> float:
    """
    Convert a raw evdev axis value to a float in [-1.0, 1.0].

    Analog sticks: apply dead zone then scale by ±32767.
    D-pad hat:     discrete -1 / 0 / 1, pass through as float.
    """
    if code in STICK_AXES:
        if abs(raw) <= DEAD_ZONE:
            return 0.0
        return max(-1.0, min(1.0, raw / STICK_MAX))

    if code in HAT_AXES:
        return float(max(-1, min(1, raw)))   # already -1/0/1, guard-clamp

    # Unknown axis – pass through clamped (shouldn't happen with CHANNEL_MAP)
    log.debug("_normalise: unknown axis code %d, value %d", code, raw)
    return max(-1.0, min(1.0, raw / STICK_MAX))


# ── Dispatcher ────────────────────────────────────────────────────────────────

class Dispatcher:
    """
    Bridges GamepadReader → AppState → SerialBridge in a background thread.

    Parameters
    ----------
    reader : GamepadReader
        Must already be started (``reader.start()`` called).
    state : AppState
        Shared application state singleton.
    bridge : SerialBridge
        Must already be open (``bridge.open()`` called), OR pass
        ``auto_open=True`` to let the dispatcher open it.
    auto_open : bool
        If True, the dispatcher calls ``bridge.open()`` before the loop
        starts and ``bridge.close()`` after it exits.
    """

    def __init__(self, reader, state, bridge, *, auto_open: bool = False):
        self._reader     = reader
        self._state      = state
        self._bridge     = bridge
        self._auto_open  = auto_open
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        # 2-byte button bitmask, see BUTTON_BIT_MAP above.
        self._button_lock = threading.Lock()
        self._buttons = bytearray(2)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the dispatcher thread."""
        if self._thread and self._thread.is_alive():
            log.warning("Dispatcher already running.")
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="dispatcher",
            daemon=True,
        )
        self._thread.start()
        log.info("Dispatcher started.")

    def stop(self, timeout: float = 2.0) -> None:
        """Signal the dispatcher thread to stop and wait for it."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=timeout)
        log.info("Dispatcher stopped.")

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def button_snapshot(self) -> bytes:
        """Thread-safe read of the current 2-byte button bitmask (e.g. for HUD display)."""
        with self._button_lock:
            return bytes(self._buttons)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _run(self) -> None:
        if self._auto_open:
            self._bridge.open()

        log.info("Dispatcher loop running.")

        try:
            while not self._stop_event.is_set():
                try:
                    event = self._reader.queue.get(timeout=QUEUE_TIMEOUT)
                except queue.Empty:
                    continue

                self._handle_event(event)

        finally:
            if self._auto_open:
                self._bridge.close()
            log.info("Dispatcher loop exited.")

    def _handle_event(self, event: dict) -> None:
        """
        Process a single event dict from GamepadReader.

        Expected shapes::

            {"type": "axis", "code": <int>, "value": <int>}   # analog stick / d-pad
            {"type": "key",  "code": <int>, "value": <int>}   # button (0=up,1=down,2=hold)

        Both axis and button events update shared state and trigger a
        transmit — the vehicle needs to react to button presses just as
        promptly as stick movement.
        """
        etype = event.get("type")

        if etype == "axis":
            self._handle_axis(event)
        elif etype == "key":
            self._handle_key(event)
        else:
            # sync / other events – not consumed here.
            return

        self._transmit()

    def _handle_axis(self, event: dict) -> None:
        code  = event["code"]
        raw   = event["value"]
        value = _normalise(code, raw)

        # Map axis code → channel index via AppState's CHANNEL_MAP.
        channel = self._state.CHANNEL_MAP.get(code)
        if channel is None:
            log.debug("Dispatcher: unmapped axis code %d, skipping.", code)
            return

        self._state.update_channel(channel, value)

    def _handle_key(self, event: dict) -> None:
        code  = event["code"]
        raw   = event["value"]
        entry = BUTTON_BIT_MAP.get(code)
        if entry is None:
            log.debug("Dispatcher: unmapped button code %d, skipping.", code)
            return

        byte_index, bit_mask = entry
        pressed = raw != 0  # evdev: 0=up, 1=down, 2=hold(repeat) — 1 and 2 both count as pressed

        with self._button_lock:
            if pressed:
                self._buttons[byte_index] |= bit_mask
            else:
                self._buttons[byte_index] &= ~bit_mask & 0xFF

    def _transmit(self) -> None:
        """Send the current channel + button snapshot over serial, if a vehicle is selected."""
        snapshot = self._state.snapshot()
        if snapshot["selected_vehicle"] is None:
            return

        buttons = self.button_snapshot()

        ok = self._bridge.write_channels(snapshot["channels"], buttons)
        if not ok:
            log.warning("Serial write failed – attempting reconnect.")
            self._reconnect()

    # ── Reconnect ─────────────────────────────────────────────────────────────

    def _reconnect(self) -> None:
        """
        Blocking reconnect loop.  Keeps trying until the port reopens or
        the dispatcher is asked to stop.
        """
        while not self._stop_event.is_set():
            try:
                self._bridge.reconnect()
                log.info("Serial reconnected.")
                return
            except Exception as exc:
                log.warning("Reconnect failed (%s) – retrying in %.1fs", exc, RECONNECT_DELAY)
                time.sleep(RECONNECT_DELAY)


"""
═══════════════════════════════════════════════════════════════════════════
REQUIRED CHANGE — serial_bridge.py
═══════════════════════════════════════════════════════════════════════════
I don't have serial_bridge.py in this session, so I couldn't edit it
directly. `write_channels()` now needs a second parameter (the 2-byte
button bitmask) and must frame a 10-byte packet instead of 8:

    < ch0 ch1 ch2 ch3 ch4 ch5 btn0 btn1 >

where each ch is a single byte 0-254 (centre=127), matching what
tx_firmware.ino expects. If write_channels currently does something like:

    def write_channels(self, channels: list[float]) -> bool:
        payload = bytes(
            int((v + 1.0) / 2.0 * 254) for v in channels
        )
        frame = b'<' + payload + b'>'
        ...

...the fix is just to extend the signature and frame:

    def write_channels(self, channels: list[float], buttons: bytes) -> bool:
        payload = bytes(
            int((v + 1.0) / 2.0 * 254) for v in channels
        )
        frame = b'<' + payload + bytes(buttons) + b'>'
        with self._lock:
            if not self.is_connected:
                return False
            try:
                self._serial.write(frame)
                return True
            except serial.serialutil.SerialException as exc:
                log.error("write_channels error: %s", exc)
                self._serial.close()
                self._serial = None
                return False

Paste in your actual serial_bridge.py and I'll make the precise edit —
I'm guessing at the packing/framing details above since I haven't seen
the real implementation.
═══════════════════════════════════════════════════════════════════════════
"""
