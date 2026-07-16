# dispatcher.py
"""
Phase 4 – Dispatcher
Connects the three subsystems:

    GamepadReader  →  AppState  →  SerialBridge

The dispatcher runs a single daemon thread that:
  1. Drains events from GamepadReader's queue.
  2. Normalises raw evdev axis values to [-1.0, 1.0].
  3. Applies a dead zone to analog sticks (values snap to 0.0 inside it).
  4. Updates AppState via update_channel().
  5. Transmits the full channel snapshot over SerialBridge — but only
     when a vehicle is currently selected.

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

        Expected shape::

            {
                "type": "axis",          # only type the dispatcher cares about
                "code": <int>,           # evdev axis code
                "value": <int>,          # raw hardware value
            }

        Button events (type == "key") are delivered to AppState in future
        phases; currently ignored here so the serial path stays focused.
        """
        if event.get("type") != "axis":
            # Button / sync / other events – not consumed here yet.
            return

        code  = event["code"]
        raw   = event["value"]
        value = _normalise(code, raw)

        # Map axis code → channel index via AppState's CHANNEL_MAP.
        channel = self._state.CHANNEL_MAP.get(code)
        if channel is None:
            log.debug("Dispatcher: unmapped axis code %d, skipping.", code)
            return

        # Update shared state.
        self._state.update_channel(channel, value)

        # Transmit only when a vehicle is selected.
        snapshot = self._state.snapshot()
        if snapshot["selected_vehicle"] is None:
            return

        ok = self._bridge.write_channels(snapshot["channels"])
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
