# gamepad.py
from evdev import InputDevice, categorize, ecodes
import queue
import threading
import logging

log = logging.getLogger(__name__)

# Axis map derived from device capabilities (Evdev Switch classification)
# Sticks range: -32767 to 32767 | D-pad range: -1 to 1
AXIS_MAP = {
    ecodes.ABS_X:     "Left Stick X",
    ecodes.ABS_Y:     "Left Stick Y",
    ecodes.ABS_RX:    "Right Stick X",
    ecodes.ABS_RY:    "Right Stick Y",
    ecodes.ABS_HAT0X: "D-Pad X",
    ecodes.ABS_HAT0Y: "D-Pad Y",
}

DEAD_ZONE = 500


class GamepadReader:
    """
    Reads evdev events from a gamepad in a background daemon thread
    and places them on a queue for the dispatcher to consume.

    Events are dicts:
        {"type": "axis", "code": int, "value": int}
        {"type": "key",  "code": int, "value": int}

    Usage::

        reader = GamepadReader("/dev/input/event5")
        reader.start()
        event = reader.queue.get()
    """

    def __init__(self, device_path: str = "/dev/input/event5"):
        self._device_path = device_path
        self._thread: threading.Thread | None = None
        self.queue: queue.Queue = queue.Queue()

    def start(self) -> None:
        """Start the background reader thread."""
        self._thread = threading.Thread(
            target=self._run,
            name="gamepad-reader",
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        try:
            device = InputDevice(self._device_path)
            log.info("Gamepad connected: %s", device.name)

            for event in device.read_loop():
                if event.type == ecodes.EV_KEY:
                    self.queue.put({
                        "type":  "key",
                        "code":  event.code,
                        "value": event.value,
                    })
                elif event.type == ecodes.EV_ABS:
                    self.queue.put({
                        "type":  "axis",
                        "code":  event.code,
                        "value": event.value,
                    })

        except PermissionError:
            log.error("Permission denied on %s — try adding pi to the 'input' group.", self._device_path)
        except FileNotFoundError:
            log.error("Gamepad not found at %s — check connection.", self._device_path)


# ── Standalone test (python3 gamepad.py) ─────────────────────────────────────
if __name__ == "__main__":
    import sys
    logging.basicConfig(level=logging.INFO)

    def format_axis_value(code, value):
        if code in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
            direction = {-1: "Negative", 0: "Center", 1: "Positive"}.get(value, str(value))
            return f"{value:+d} ({direction})"
        else:
            tag = "DEAD ZONE" if abs(value) <= DEAD_ZONE else f"{value / 32767 * 100:+.1f}%"
            return f"{value:+6d} [{tag}]"

    path = sys.argv[1] if len(sys.argv) > 1 else "/dev/input/event5"
    reader = GamepadReader(path)
    reader.start()
    print(f"Listening on {path} — Ctrl+C to exit.")

    try:
        while True:
            ev = reader.queue.get()
            if ev["type"] == "key":
                state = {1: "Pressed", 0: "Released", 2: "Held"}.get(ev["value"], "?")
                print(f"Key {state}: code={ev['code']}")
            elif ev["type"] == "axis":
                name = AXIS_MAP.get(ev["code"], f"axis{ev['code']}")
                print(f"Axis [{name:>14}]: {format_axis_value(ev['code'], ev['value'])}")
    except KeyboardInterrupt:
        print("\nExiting.")
