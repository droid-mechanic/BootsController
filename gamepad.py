# gamepad.py
from evdev import InputDevice, ecodes, list_devices
import logging
import os
import queue
import threading

log = logging.getLogger(__name__)

MAX_PLAYERS   = 2
SCAN_INTERVAL = 1.5   # seconds between device scans
# Optional: only accept devices whose name contains this (case-insensitive).
NAME_FILTER   = os.getenv("GAMEPAD_NAME_FILTER", "").lower()

AXIS_MAP = {
    ecodes.ABS_X:     "Left Stick X",
    ecodes.ABS_Y:     "Left Stick Y",
    ecodes.ABS_RX:    "Right Stick X",
    ecodes.ABS_RY:    "Right Stick Y",
    ecodes.ABS_HAT0X: "D-Pad X",
    ecodes.ABS_HAT0Y: "D-Pad Y",
}

DEAD_ZONE = 500


def _is_gamepad(dev: InputDevice) -> bool:
    """A gamepad has face buttons AND a left stick (excludes IMU, audio, keyboards)."""
    caps = dev.capabilities()
    keys = caps.get(ecodes.EV_KEY, [])
    axes = [code for code, _info in caps.get(ecodes.EV_ABS, [])]
    if ecodes.BTN_SOUTH not in keys or ecodes.ABS_X not in axes:
        return False
    if NAME_FILTER and NAME_FILTER not in (dev.name or "").lower():
        return False
    return True


def _device_key(dev: InputDevice) -> str:
    """Stable identity: Bluetooth MAC (uniq) when available."""
    return dev.uniq or dev.phys or dev.path


class GamepadManager:
    """
    Discovers gamepads, assigns each a stable player slot (0..max_players-1),
    and puts tagged events on a single shared queue.

    Events (dicts):
        {"player": int, "type": "axis",       "code": int, "value": int}
        {"player": int, "type": "key",        "code": int, "value": int}
        {"player": int, "type": "connect",    "name": str}
        {"player": int, "type": "disconnect"}
    """

    def __init__(self, max_players: int = MAX_PLAYERS):
        self.queue: queue.Queue = queue.Queue()
        self._max = max_players
        self._lock = threading.Lock()
        # slot -> {"key", "name", "path", "connected"} or None
        self._slots: list[dict | None] = [None] * max_players
        self._active_paths: set[str] = set()
        self._rejected: set[str] = set()     # paths known not to be gamepads
        self._warned_full = False
        self._stop = threading.Event()
        self._scanner: threading.Thread | None = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._scanner and self._scanner.is_alive():
            return
        self._stop.clear()
        self._scanner = threading.Thread(
            target=self._scan_loop, name="gamepad-scanner", daemon=True
        )
        self._scanner.start()

    def stop(self) -> None:
        self._stop.set()

    def status(self) -> list[dict]:
        """Snapshot for the UI: one entry per slot."""
        with self._lock:
            return [
                {
                    "player": i,
                    "connected": bool(s and s["connected"]),
                    "name": s["name"] if s else None,
                }
                for i, s in enumerate(self._slots)
            ]

    # ── Discovery ─────────────────────────────────────────────────────────────

    def _scan_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._scan_once()
            except Exception:
                log.exception("Gamepad scan failed")
            self._stop.wait(SCAN_INTERVAL)

    def _scan_once(self) -> None:
        paths = set(list_devices())
        self._rejected &= paths            # forget nodes that vanished

        for path in sorted(paths - self._active_paths - self._rejected):
            try:
                dev = InputDevice(path)
            except OSError:
                continue                   # permissions / race with unplug

            if not _is_gamepad(dev):
                self._rejected.add(path)
                dev.close()
                continue

            player = self._claim_slot(dev)
            if player is None:
                dev.close()
                if not self._warned_full:
                    log.warning("Gamepad %s ignored: all %d slots in use.", dev.name, self._max)
                    self._warned_full = True
                continue

            self._warned_full = False
            self._active_paths.add(path)
            threading.Thread(
                target=self._read_device,
                args=(dev, player),
                name=f"gamepad-p{player}",
                daemon=True,
            ).start()

    def _claim_slot(self, dev: InputDevice) -> int | None:
        key = _device_key(dev)
        with self._lock:
            # 1. Same controller reconnecting -> its old slot.
            for i, s in enumerate(self._slots):
                if s and s["key"] == key:
                    if s["connected"]:
                        return None        # already open via another node
                    s.update(name=dev.name, path=dev.path, connected=True)
                    return i
            # 2. Empty slot, 3. reclaim a disconnected one.
            order = [i for i, s in enumerate(self._slots) if s is None] + \
                    [i for i, s in enumerate(self._slots) if s and not s["connected"]]
            if not order:
                return None
            i = order[0]
            self._slots[i] = {"key": key, "name": dev.name, "path": dev.path, "connected": True}
            return i

    # ── Per-device reader ─────────────────────────────────────────────────────

    def _read_device(self, dev: InputDevice, player: int) -> None:
        log.info("Player %d connected: %s (%s)", player + 1, dev.name, dev.path)
        self.queue.put({"player": player, "type": "connect", "name": dev.name})
        try:
            for event in dev.read_loop():
                if event.type == ecodes.EV_KEY:
                    self.queue.put({"player": player, "type": "key",
                                    "code": event.code, "value": event.value})
                elif event.type == ecodes.EV_ABS:
                    self.queue.put({"player": player, "type": "axis",
                                    "code": event.code, "value": event.value})
        except OSError as exc:             # ENODEV when the controller drops
            log.info("Player %d read ended: %s", player + 1, exc)
        finally:
            try:
                dev.close()
            except Exception:
                pass
            with self._lock:
                if self._slots[player]:
                    self._slots[player]["connected"] = False
            self._active_paths.discard(dev.path)
            self.queue.put({"player": player, "type": "disconnect"})
            log.warning("Player %d disconnected.", player + 1)


# ── Standalone test (python3 gamepad.py) ─────────────────────────────────────
if __name__ == "__main__":
    import time
    logging.basicConfig(level=logging.INFO)

    def format_axis_value(code, value):
        if code in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
            direction = {-1: "Negative", 0: "Center", 1: "Positive"}.get(value, str(value))
            return f"{value:+d} ({direction})"
        tag = "DEAD ZONE" if abs(value) <= DEAD_ZONE else f"{value / 32767 * 100:+.1f}%"
        return f"{value:+6d} [{tag}]"

    mgr = GamepadManager()
    mgr.start()
    print("Listening for up to 2 gamepads — Ctrl+C to exit.")
    try:
        while True:
            try:
                ev = mgr.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            p = f"P{ev['player'] + 1}"
            if ev["type"] == "key":
                st = {1: "Pressed", 0: "Released", 2: "Held"}.get(ev["value"], "?")
                print(f"{p} Key {st}: code={ev['code']}")
            elif ev["type"] == "axis":
                name = AXIS_MAP.get(ev["code"], f"axis{ev['code']}")
                print(f"{p} Axis [{name:>14}]: {format_axis_value(ev['code'], ev['value'])}")
            else:
                print(f"{p} {ev['type']}")
    except KeyboardInterrupt:
        print("\nExiting.")