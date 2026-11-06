# dispatcher.py
"""
Dispatcher (two-player)
Connects the three subsystems:

    GamepadManager  →  AppState  →  SerialBridge

The dispatcher runs a single daemon thread that:
  1. Drains tagged events from GamepadManager's queue.
  2. Normalises raw evdev axis values to [-1.0, 1.0] and applies a dead zone.
  3. Tracks each player's buttons as a 2-byte bitmask (see BUTTON_BIT_MAP),
     matching the layout expected by tx_firmware.ino / the vehicle firmware.
  4. Updates AppState (per-player channels, controller connect/disconnect).
  5. At a fixed rate (TX_HZ), sends one control packet per player that holds
     a vehicle. Transmitting on a timer rather than on every input event
     keeps the vehicles' 500 ms failsafe fed even when the sticks are held
     still, and caps serial/radio traffic regardless of gamepad poll rate.
  6. Keeps the transmitter's peer table in step with AppState: when a
     player's vehicle changes it sends the new peer; every PEER_REFRESH
     seconds it re-sends all peers, so a transmitter that reset is healed
     without any action from the operators.

Serial disconnections are detected on write failure and trigger a blocking
reconnect inside the thread (the event queue keeps filling safely).
"""

import logging
import queue
import threading
import time

from evdev import ecodes

from state import NUM_PLAYERS

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

# ── Timing ────────────────────────────────────────────────────────────────────

TX_HZ          = 50                # control packets per second, per player
TX_PERIOD      = 1.0 / TX_HZ
PEER_REFRESH   = 2.0               # seconds between full peer re-sends
RECONNECT_DELAY = 2.0              # seconds between serial reconnect attempts
QUEUE_TIMEOUT   = 0.1              # keeps the thread responsive to stop()

_UNSET = object()   # "unknown what the transmitter has" (vs None = "cleared")

# ── Button bit layout ──────────────────────────────────────────────────────
# Must match the ControlPayload.buttons[] layout in tx_firmware.ino and the
# vehicle firmware:
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
    Bridges GamepadManager → AppState → SerialBridge in a background thread.

    Parameters
    ----------
    reader : GamepadManager
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

        # One 2-byte button bitmask per player, see BUTTON_BIT_MAP above.
        self._button_lock = threading.Lock()
        self._buttons = [bytearray(2) for _ in range(NUM_PLAYERS)]

        # What the transmitter's slot table is believed to hold, per slot:
        # a MAC string, None (cleared), or _UNSET (unknown → resend).
        self._peer_sent: list = [_UNSET] * NUM_PLAYERS

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

    def button_snapshot(self, player: int = 0) -> bytes:
        """Thread-safe read of a player's 2-byte button bitmask."""
        with self._button_lock:
            return bytes(self._buttons[player])

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _run(self) -> None:
        log.info("Dispatcher thread started, reader queue id: %s", id(self._reader.queue))
        if self._auto_open:
            self._bridge.open()

        next_tx      = time.monotonic()
        next_refresh = next_tx

        try:
            while not self._stop_event.is_set():
                # Wait for an event, but never past the next transmit tick.
                timeout = max(0.0, min(next_tx - time.monotonic(), QUEUE_TIMEOUT))
                try:
                    event = self._reader.queue.get(timeout=timeout)
                except queue.Empty:
                    event = None

                if event is not None:
                    self._handle_event(event)

                now = time.monotonic()
                if now >= next_tx:
                    refresh = now >= next_refresh
                    if refresh:
                        next_refresh = now + PEER_REFRESH
                    self._tick(force_peers=refresh)
                    next_tx = now + TX_PERIOD
        finally:
            self._release_all()
            if self._auto_open:
                self._bridge.close()
            log.info("Dispatcher loop exited.")

    # ── Input handling ────────────────────────────────────────────────────────

    def _handle_event(self, event: dict) -> None:
        """
        Process a single event dict from GamepadManager::

            {"player": n, "type": "axis",       "code": <int>, "value": <int>}
            {"player": n, "type": "key",        "code": <int>, "value": <int>}
            {"player": n, "type": "connect",    "name": <str>}
            {"player": n, "type": "disconnect"}

        Events only update state; transmission happens on the timer.
        """
        player = event.get("player", 0)
        if not 0 <= player < NUM_PLAYERS:
            return
        etype = event.get("type")

        if etype == "axis":
            self._handle_axis(player, event)
        elif etype == "key":
            self._handle_key(player, event)
        elif etype == "connect":
            self._state.set_controller(player, True, event.get("name"))
        elif etype == "disconnect":
            # Never leave a stuck stick or button: AppState zeroes the axes,
            # we zero the buttons. The vehicle keeps receiving neutral frames.
            with self._button_lock:
                self._buttons[player][:] = bytes(2)
            self._state.set_controller(player, False)

    def _handle_axis(self, player: int, event: dict) -> None:
        code  = event["code"]
        value = _normalise(code, event["value"])

        # Map axis code → channel index via AppState's CHANNEL_MAP.
        channel = self._state.CHANNEL_MAP.get(code)
        if channel is None:
            log.debug("Dispatcher: unmapped axis code %d, skipping.", code)
            return
        self._state.update_channel(channel, value, player)

    def _handle_key(self, player: int, event: dict) -> None:
        code  = event["code"]
        entry = BUTTON_BIT_MAP.get(code)
        if entry is None:
            log.debug("Dispatcher: unmapped button code %d, skipping.", code)
            return

        byte_index, bit_mask = entry
        pressed = event["value"] != 0  # evdev: 0=up, 1=down, 2=hold(repeat)

        with self._button_lock:
            if pressed:
                self._buttons[player][byte_index] |= bit_mask
            else:
                self._buttons[player][byte_index] &= ~bit_mask & 0xFF

    # ── Transmit ──────────────────────────────────────────────────────────────

    def _tick(self, force_peers: bool) -> None:
        """One transmit cycle: sync peers, then one packet per active player."""
        players = self._state.snapshot()["players"]

        if not self._sync_peers(players, force_peers):
            return

        for pl in players:
            if pl["selected_vehicle"] is None:
                continue
            p  = pl["player"]
            ok = self._bridge.write_channels(p, pl["channels"], self.button_snapshot(p))
            if not ok:
                self._serial_failed()
                return

    def _sync_peers(self, players: list[dict], force: bool) -> bool:
        """
        Make the transmitter's slot table match AppState. Returns False if
        the serial link failed (a reconnect has already been attempted).
        """
        for pl in players:
            p       = pl["player"]
            vehicle = pl["selected_vehicle"]
            mac     = vehicle["mac"] if vehicle else None
            prev    = self._peer_sent[p]

            if not force and prev == mac:
                continue

            # Leaving a vehicle: send a neutral frame first so it stops now
            # instead of waiting out its 500 ms failsafe.
            if isinstance(prev, str) and prev != mac:
                self._bridge.write_channels(p, {}, bytes(2))

            if not self._bridge.send_peer_command(p, mac):
                self._serial_failed()
                return False

            if prev != mac:
                log.info("Player %d → %s", p + 1, vehicle["name"] if vehicle else "no vehicle")
            self._peer_sent[p] = mac
        return True

    def _release_all(self) -> None:
        """Best-effort on shutdown: stop and unlink every vehicle."""
        try:
            for p in range(NUM_PLAYERS):
                if isinstance(self._peer_sent[p], str):
                    self._bridge.write_channels(p, {}, bytes(2))
                self._bridge.send_peer_command(p, None)
        except Exception:
            log.debug("Release on shutdown failed.", exc_info=True)

    # ── Reconnect ─────────────────────────────────────────────────────────────

    def _serial_failed(self) -> None:
        log.warning("Serial write failed – attempting reconnect.")
        # The transmitter probably reset with the port: resend every peer.
        self._peer_sent = [_UNSET] * NUM_PLAYERS
        self._reconnect()

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
