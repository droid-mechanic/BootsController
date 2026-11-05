# state.py - Vehicle registry and shared app state (two-player)
#
# All vehicles share the same fixed channel layout - no per-vehicle axis
# remapping is needed. A Vehicle is pure metadata (name, image, target MAC).
# AppState is a singleton shared across all backend threads.
#
# Two operators ("players", indexed 0 and 1) each hold at most one vehicle,
# and a vehicle can be held by at most one player at a time.
#
# Channel layout (index → semantic meaning), per player:
#   0  Left Stick X   steering / roll
#   1  Left Stick Y   throttle / pitch
#   2  Right Stick X  yaw / camera pan
#   3  Right Stick Y  elevation / camera tilt
#   4  D-Pad X
#   5  D-Pad Y

import json
import logging
import threading
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Callable, Optional
from evdev import ecodes

log = logging.getLogger(__name__)

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
NUM_PLAYERS  = 2

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


class VehicleTaken(Exception):
    """Raised when a player tries to select a vehicle another player holds."""

    def __init__(self, vehicle: Vehicle, player: int):
        super().__init__(f"'{vehicle.name}' is already in use by player {player + 1}")
        self.vehicle = vehicle
        self.player  = player


# ── AppState singleton ────────────────────────────────────────────────────────

class AppState:
    """
    Shared application state accessed by the gamepad reader, dispatcher,
    serial bridge, and Flask server.

    Thread-safety: a reentrant lock guards all mutations. Callers that need a
    consistent view of several fields should use snapshot() or hold the lock.

    Change notification: set ``on_change`` to a callable taking a reason
    string ("selection" | "controller"). It is invoked *after* the lock is
    released, so it may safely call snapshot().
    """

    # Class-level reference so dispatcher can do state.CHANNEL_MAP
    CHANNEL_MAP = CHANNEL_MAP

    def __init__(self, vehicles_path: str | Path = "vehicles.json"):
        self.lock = threading.RLock()

        # Vehicle registry - loaded once at startup, never mutated
        self.vehicles: list[Vehicle] = load_vehicles(vehicles_path)
        self._vehicle_index: dict[str, Vehicle] = {v.id: v for v in self.vehicles}

        # Per-player state, indexed by player number (0 = left, 1 = right).
        self.selected: list[Optional[Vehicle]] = [None] * NUM_PLAYERS
        self.channels: list[list[float]] = [[0.0] * NUM_CHANNELS for _ in range(NUM_PLAYERS)]
        self.controllers: list[dict] = [
            {"connected": False, "name": None} for _ in range(NUM_PLAYERS)
        ]

        self.on_change: Optional[Callable[[str], None]] = None

    # ── Legacy single-player accessors (player 0) ─────────────────────────────
    # Kept so code written before two-player support keeps working until the
    # dispatcher is made per-player (Phase 3).

    @property
    def selected_vehicle(self) -> Optional[Vehicle]:
        return self.selected[0]

    @property
    def channel_state(self) -> list[float]:
        return self.channels[0]

    @property
    def screen(self) -> str:
        return "hud" if self.selected[0] else "selection"

    # ── Lookup ────────────────────────────────────────────────────────────────

    def get_vehicle(self, vehicle_id: str) -> Optional[Vehicle]:
        return self._vehicle_index.get(vehicle_id)

    def owner_of(self, vehicle_id: str) -> Optional[int]:
        """Player index currently holding this vehicle, or None."""
        with self.lock:
            for p, v in enumerate(self.selected):
                if v is not None and v.id == vehicle_id:
                    return p
        return None

    # ── Vehicle selection ─────────────────────────────────────────────────────

    def select(self, vehicle: Vehicle, player: int = 0) -> Vehicle:
        """
        Claim ``vehicle`` for ``player`` (releasing any vehicle they already
        held). The check-and-claim is atomic.

        Raises VehicleTaken if the other player holds it.
        """
        with self.lock:
            self._check_player(player)
            for p, held in enumerate(self.selected):
                if p != player and held is not None and held.id == vehicle.id:
                    raise VehicleTaken(vehicle, p)
            changed = self.selected[player] is not vehicle
            self.selected[player] = vehicle
            self._reset_channels(player)
        if changed:
            self._notify("selection")
        return vehicle

    def deselect(self, player: int = 0) -> None:
        """Release the player's vehicle (no-op if they hold none)."""
        with self.lock:
            self._check_player(player)
            changed = self.selected[player] is not None
            self.selected[player] = None
            self._reset_channels(player)
        if changed:
            self._notify("selection")

    # ── Controller status ─────────────────────────────────────────────────────

    def set_controller(self, player: int, connected: bool, name: Optional[str] = None) -> None:
        """Record a gamepad connecting/disconnecting for a player slot."""
        with self.lock:
            self._check_player(player)
            new = {"connected": connected, "name": name if connected else None}
            if self.controllers[player] == new:
                return
            self.controllers[player] = new
            if not connected:
                self._reset_channels(player)   # never leave a stuck stick
        self._notify("controller")

    # ── Channel state ─────────────────────────────────────────────────────────

    def update_channel(self, channel: int, value: float, player: int = 0) -> bool:
        """
        Write a normalised float [-1.0, 1.0] into a player's channel slot.
        Returns True if the channel and player indices are valid.
        """
        if channel < 0 or channel >= NUM_CHANNELS:
            return False
        if player < 0 or player >= NUM_PLAYERS:
            return False
        with self.lock:
            self.channels[player][channel] = value
        return True

    def snapshot(self) -> dict:
        """
        JSON-serializable snapshot of the current state.

        ``players`` is the source of truth. The top-level ``selected_vehicle``,
        ``channels`` and ``screen`` keys mirror player 0 for older consumers.
        """
        with self.lock:
            players = [
                {
                    "player":           p,
                    "controller":       dict(self.controllers[p]),
                    "selected_vehicle": self.selected[p].to_dict() if self.selected[p] else None,
                    "channels":         {i: v for i, v in enumerate(self.channels[p])},
                }
                for p in range(NUM_PLAYERS)
            ]
            return {
                "players":          players,
                "channel_names":    CHANNEL_NAMES,
                # legacy (player 0)
                "screen":           self.screen,
                "selected_vehicle": players[0]["selected_vehicle"],
                "channels":         players[0]["channels"],
            }

    # ── Internal ──────────────────────────────────────────────────────────────

    @staticmethod
    def _check_player(player: int) -> None:
        if not 0 <= player < NUM_PLAYERS:
            raise ValueError(f"player must be 0..{NUM_PLAYERS - 1}, got {player}")

    def _reset_channels(self, player: int) -> None:
        """Zero one player's channels (caller holds the lock)."""
        self.channels[player] = [0.0] * NUM_CHANNELS

    def _notify(self, reason: str) -> None:
        cb = self.on_change
        if cb is None:
            return
        try:
            cb(reason)
        except Exception:
            log.exception("on_change callback failed")


# ── Standalone test ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    vehicles_path = sys.argv[1] if len(sys.argv) > 1 else "vehicles.json"
    state = AppState(vehicles_path)
    events: list[str] = []
    state.on_change = events.append

    print(f"Loaded {len(state.vehicles)} vehicle(s):")
    for v in state.vehicles:
        print(f"  [{v.id}]  {v.name}  -  MAC {v.mac}")

    a, b = state.vehicles[0], state.vehicles[1]

    state.select(a, player=0)
    state.select(b, player=1)
    assert state.selected[0] is a and state.selected[1] is b
    print("\nP1 holds", a.name, "| P2 holds", b.name)

    try:
        state.select(a, player=1)
        raise SystemExit("FAIL: player 2 was allowed to take player 1's vehicle")
    except VehicleTaken as exc:
        print("Conflict rejected:", exc)
    assert state.selected[1] is b, "failed claim must not disturb P2's current vehicle"

    state.deselect(0)
    state.select(a, player=1)          # now free
    assert state.selected[1] is a and state.selected[0] is None
    print("After P1 released, P2 took", a.name)

    state.update_channel(0, 0.5, player=1)
    state.set_controller(1, True, "Pro Controller")
    state.set_controller(1, False)
    assert state.channels[1][0] == 0.0, "disconnect must zero that player's channels"

    snap = state.snapshot()
    assert snap["selected_vehicle"] is None and snap["players"][1]["selected_vehicle"]["id"] == a.id
    print("\nnotifications:", events)
    print("Two-player state test passed.")
