# state.py — Phase 2: Vehicle registry and shared app state
#
# All vehicles share the same fixed channel layout — no per-vehicle axis
# remapping is needed. A Vehicle is pure metadata (name, image, target MAC).
# AppState is a singleton shared across all backend threads.
#
# Channel layout (index → semantic meaning):
#   0  Left Stick X   steering / roll
#   1  Left Stick Y   throttle / pitch
#   2  Right Stick X  yaw / camera pan
#   3  Right Stick Y  elevation / camera tilt
#   4  D-Pad X
#   5  D-Pad Y

import json
import threading
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional
from evdev import ecodes

# ── Channel definitions ───────────────────────────────────────────────────────

# Maps evdev axis code → channel index in the serialized packet.
# Fixed for all vehicles.
CHANNEL_MAP = {
    ecodes.ABS_X:     0,   # Left Stick X
    ecodes.ABS_Y:     1,   # Left Stick Y
    ecodes.ABS_RX:    2,   # Right Stick X
    ecodes.ABS_RY:    3,   # Right Stick Y
    ecodes.ABS_HAT0X: 4,   # D-Pad X
    ecodes.ABS_HAT0Y: 5,   # D-Pad Y
}

CHANNEL_NAMES = [
    "Left Stick X",
    "Left Stick Y",
    "Right Stick X",
    "Right Stick Y",
    "D-Pad X",
    "D-Pad Y",
]

NUM_CHANNELS = len(CHANNEL_NAMES)

# ── Vehicle dataclass ─────────────────────────────────────────────────────────

@dataclass
class Vehicle:
    id:    str
    name:  str
    image: str
    mac:   str

    @classmethod
    def from_dict(cls, data: dict) -> "Vehicle":
        return cls(
            id    = data["id"],
            name  = data["name"],
            image = data["image"],
            # accept either key name from vehicles.json
            mac   = data.get("mac") or data.get("esp32_mac", ""),
        )

    def to_dict(self) -> dict:
        return asdict(self)


def load_vehicles(path: str | Path = "vehicles.json") -> list[Vehicle]:
    """Parse vehicles.json and return a list of Vehicle instances."""
    with open(path) as f:
        entries = json.load(f)
    return [Vehicle.from_dict(e) for e in entries]


# ── AppState singleton ────────────────────────────────────────────────────────

class AppState:
    """
    Shared application state accessed by the gamepad reader, dispatcher,
    serial bridge, and Flask server.

    Thread-safety: a reentrant lock guards all mutations. Reads of simple
    scalar fields (screen, selected_vehicle) are safe without the lock on
    CPython due to the GIL, but callers that need a consistent snapshot of
    multiple fields should use the lock explicitly:

        with state.lock:
            vehicle = state.selected_vehicle
            channels = state.channel_state.copy()
    """

    # Class-level reference so dispatcher can do state.CHANNEL_MAP
    CHANNEL_MAP = CHANNEL_MAP

    def __init__(self, vehicles_path: str | Path = "vehicles.json"):
        self.lock = threading.RLock()

        # Vehicle registry — loaded once at startup, never mutated
        self.vehicles: list[Vehicle] = load_vehicles(vehicles_path)
        self._vehicle_index: dict[str, Vehicle] = {v.id: v for v in self.vehicles}

        # Active vehicle (None = selection screen)
        self.selected_vehicle: Optional[Vehicle] = None

        # Current UI screen
        self.screen: str = "selection"   # "selection" | "hud"

        # Live channel values — updated by the dispatcher each event.
        # Index matches CHANNEL_MAP / CHANNEL_NAMES.
        self.channel_state: list[int] = [0] * NUM_CHANNELS

    # ── Vehicle selection ─────────────────────────────────────────────────────

    def select(self, vehicle: "Vehicle") -> "Vehicle":
        """
        Set the active vehicle and transition to the HUD screen.
        Accepts a Vehicle object (server looks up the vehicle before calling).
        """
        with self.lock:
            self.selected_vehicle = vehicle
            self.screen = "hud"
            self._reset_channels()
        return vehicle

    def deselect(self) -> None:
        """Clear the active vehicle and return to the selection screen."""
        with self.lock:
            self.selected_vehicle = None
            self.screen = "selection"
            self._reset_channels()

    # ── Channel state ─────────────────────────────────────────────────────────

    def update_channel(self, channel: int, value: float) -> bool:
        """
        Write a normalised float [-1.0, 1.0] into a channel slot by index.
        Called by the dispatcher after it has resolved evdev_code → channel
        via CHANNEL_MAP and normalised the raw value.

        Returns True if the channel index is valid, False otherwise.
        """
        if channel < 0 or channel >= NUM_CHANNELS:
            return False
        with self.lock:
            self.channel_state[channel] = value
        return True

    def snapshot(self) -> dict:
        """
        Return a JSON-serializable snapshot of the current state — used by
        the Flask WebSocket broadcaster.
        """
        with self.lock:
            return {
                "screen":           self.screen,
                "selected_vehicle": self.selected_vehicle.to_dict()
                                    if self.selected_vehicle else None,
                "channels":         {i: v for i, v in enumerate(self.channel_state)},
                "channel_names":    CHANNEL_NAMES,
            }

    # ── Internal ──────────────────────────────────────────────────────────────

    def _reset_channels(self) -> None:
        """Zero all channel values (called without acquiring lock — caller holds it)."""
        self.channel_state = [0] * NUM_CHANNELS


# ── Standalone test ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    vehicles_path = sys.argv[1] if len(sys.argv) > 1 else "vehicles.json"

    print(f"Loading vehicles from: {vehicles_path}\n")
    state = AppState(vehicles_path)

    print(f"Loaded {len(state.vehicles)} vehicle(s):")
    for v in state.vehicles:
        print(f"  [{v.id}]  {v.name}  —  MAC {v.esp32_mac}  —  image: {v.image}")

    print()

    # Test select / deselect
    first_id = state.vehicles[0].id
    print(f"Selecting '{first_id}'…")
    state.select(first_id)
    print(f"  screen={state.screen}  selected={state.selected_vehicle.name}")

    # Test channel update
    print("\nSimulating axis events:")
    test_events = [
        (ecodes.ABS_X,     15000),   # Left Stick X
        (ecodes.ABS_Y,    -8000),    # Left Stick Y
        (ecodes.ABS_RX,    3200),    # Right Stick X
        (ecodes.ABS_RY,   -3200),    # Right Stick Y
        (ecodes.ABS_HAT0X,    1),    # D-Pad right
        (ecodes.ABS_HAT0Y,    0),    # D-Pad centred
        (999,                 0),    # unknown code — should return False
    ]
    for code, value in test_events:
        applied = state.update_channel(code, value)
        name = CHANNEL_NAMES[CHANNEL_MAP[code]] if code in CHANNEL_MAP else "UNKNOWN"
        print(f"  code={code:>3}  value={value:+6d}  channel={name:<16}  applied={applied}")

    print(f"\nchannel_state: {state.channel_state}")

    print("\nsnapshot():")
    snap = state.snapshot()
    for k, v in snap.items():
        print(f"  {k}: {v}")

    # Test deselect
    print("\nDeselecting…")
    state.deselect()
    print(f"  screen={state.screen}  selected={state.selected_vehicle}")
    print(f"  channels reset: {state.channel_state}")

    # Test bad id
    print("\nTesting invalid vehicle id…")
    try:
        state.select("does_not_exist")
    except KeyError as e:
        print(f"  KeyError raised as expected: {e}")

    print("\nPhase 2 test complete.")
