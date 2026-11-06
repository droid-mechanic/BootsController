# server.py
"""
Flask + WebSocket Backend (two-player)
======================================
HTTP REST + WebSocket server that bridges the frontend UI to the
controller pipeline (GamepadManager → Dispatcher → SerialBridge).

REST endpoints
--------------
GET  /api/vehicles                          → JSON list of all vehicles
GET  /api/state                             → JSON snapshot of current AppState
POST /api/players/<n>/select/<vehicle_id>   → Player n (0 or 1) claims a vehicle.
                                              200 + state on success,
                                              409 + {error, taken_by, state} if the
                                              other player already holds it.
POST /api/players/<n>/deselect              → Player n releases their vehicle.

Legacy aliases for player 0 (older frontends):
POST /api/select/<vehicle_id>
POST /api/deselect

WebSocket
---------
ws://host/ws

The server pushes a JSON state snapshot to every connected client whenever
a vehicle is claimed/released or a controller connects/disconnects.

Message format (server → client):
    {
        "event":   "state_update",
        "players": [
            {
                "player":           0,
                "controller":       { "connected": bool, "name": str | null },
                "selected_vehicle": { id, name, image, mac } | null,
                "channels":         { "0": float, "1": float, … }
            },
            { … player 1 … }
        ],
        # legacy mirrors of players[0]:
        "selected_vehicle": …,
        "channels": …
    }

No messages are expected from the client over WebSocket; it is
receive-only. Selection is done via the REST endpoints.

Static files
------------
Served from ./static/.  index.html is served at /.
"""

import json
import logging
import threading
import time

from flask import Flask, jsonify, send_from_directory, abort
from flask_sock import Sock

from simple_websocket import ConnectionClosed

from state import NUM_PLAYERS, VehicleTaken

log = logging.getLogger(__name__)

# ── Tunables ──────────────────────────────────────────────────────────────────

# Minimum seconds between WebSocket pushes triggered by axis events.
# (Axis pushes are not currently wired: AppState only notifies on selection and
# controller changes. Kept for when the HUD needs live stick values.)
WS_AXIS_INTERVAL = 0.05   # 20 Hz max to clients

# Directory that Flask serves static assets from.
STATIC_DIR = "static"


# ── Server class ──────────────────────────────────────────────────────────────

class ControllerServer:
    """
    Encapsulates the Flask app and WebSocket hub.

    Parameters
    ----------
    state   : AppState         – shared application state
    reader  : GamepadManager   – started gamepad manager
    bridge  : SerialBridge     – open serial bridge
    dispatcher : Dispatcher    – started dispatcher
    vehicles   : list[Vehicle] – loaded vehicle list (from load_vehicles())
    """

    def __init__(self, state, reader, bridge, dispatcher, vehicles):
        self._state      = state
        self._reader     = reader
        self._bridge     = bridge
        self._dispatcher = dispatcher
        self._vehicles   = vehicles

        # WebSocket client registry {sock: threading.Event}
        # The event is set when the connection should close.
        self._ws_clients: dict = {}
        self._ws_lock = threading.Lock()

        # Axis-push throttle state
        self._last_axis_push = 0.0

        # Build Flask app
        self.app = Flask(__name__, static_folder=STATIC_DIR)
        self._sock = Sock(self.app)
        self._register_routes()

        # AppState calls this whenever selection or controller status changes.
        self._state.on_change = self._on_state_change

    # ── Route registration ────────────────────────────────────────────────────

    def _register_routes(self) -> None:
        app  = self.app
        sock = self._sock

        # ── Static ────────────────────────────────────────────────────────────

        @app.route("/")
        def index():
            return send_from_directory(STATIC_DIR, "index.html")

        @app.route("/<path:filename>")
        def static_files(filename):
            return send_from_directory(STATIC_DIR, filename)

        # ── REST ──────────────────────────────────────────────────────────────

        @app.route("/api/vehicles")
        def api_vehicles():
            return jsonify([_vehicle_dict(v) for v in self._vehicles])

        @app.route("/api/state")
        def api_state():
            return jsonify(self._public_snapshot())

        @app.route("/api/players/<int:player>/select/<vehicle_id>", methods=["POST"])
        def api_player_select(player, vehicle_id):
            return self._select(player, vehicle_id)

        @app.route("/api/players/<int:player>/deselect", methods=["POST"])
        def api_player_deselect(player):
            return self._deselect(player)

        # Legacy single-player aliases → player 0
        @app.route("/api/select/<vehicle_id>", methods=["POST"])
        def api_select(vehicle_id):
            return self._select(0, vehicle_id)

        @app.route("/api/deselect", methods=["POST"])
        def api_deselect():
            return self._deselect(0)

        # ── WebSocket ─────────────────────────────────────────────────────────

        @sock.route("/ws")
        def ws_handler(ws):
            """
            One long-lived connection per client.
            We register the socket, send an immediate state snapshot,
            then block until the connection closes or we signal it.
            """
            close_event = threading.Event()

            with self._ws_lock:
                self._ws_clients[ws] = close_event

            log.info("WebSocket client connected. Total: %d", len(self._ws_clients))

            try:
                # Immediately sync the new client.
                self._send(ws, self._public_snapshot())

                # Keep the handler alive; flask-sock closes when we return.
                # We wake on close_event to allow clean server shutdown.
                while not close_event.is_set():
                    # recv with timeout so we can check close_event.
                    # Clients send nothing, so this is just a keep-alive poll.
                    try:
                        ws.receive(timeout=1)
                    except ConnectionClosed:
                        break
            finally:
                with self._ws_lock:
                    self._ws_clients.pop(ws, None)
                log.info("WebSocket client disconnected. Total: %d", len(self._ws_clients))

    # ── Selection logic ───────────────────────────────────────────────────────

    def _select(self, player: int, vehicle_id: str):
        if not 0 <= player < NUM_PLAYERS:
            abort(404, description=f"Player {player} does not exist.")
        vehicle = self._find_vehicle(vehicle_id)
        if vehicle is None:
            abort(404, description=f"Vehicle '{vehicle_id}' not found.")

        try:
            self._state.select(vehicle, player)
        except VehicleTaken as exc:
            return jsonify({
                "error":    str(exc),
                "taken_by": exc.player,
                "state":    self._public_snapshot(),
            }), 409

        # The dispatcher watches AppState and points the transmitter at the
        # player's vehicle; nothing to send from here.
        return jsonify(self._public_snapshot())

    def _deselect(self, player: int):
        if not 0 <= player < NUM_PLAYERS:
            abort(404, description=f"Player {player} does not exist.")
        self._state.deselect(player)
        return jsonify(self._public_snapshot())

    # ── State change hook ─────────────────────────────────────────────────────

    def _on_state_change(self, reason: str = "axis") -> None:
        """
        Called by AppState when state mutates. Axis-triggered pushes are
        throttled to WS_AXIS_INTERVAL; selection/controller changes push
        immediately.
        """
        now = time.monotonic()
        if reason == "axis":
            if now - self._last_axis_push < WS_AXIS_INTERVAL:
                return
            self._last_axis_push = now

        self._push_to_all(self._public_snapshot())

    # ── WebSocket helpers ─────────────────────────────────────────────────────

    def _push_to_all(self, payload: dict) -> None:
        """Broadcast a dict as JSON to every connected WebSocket client."""
        message = json.dumps(payload)
        dead = []

        with self._ws_lock:
            clients = list(self._ws_clients.keys())

        for ws in clients:
            try:
                ws.send(message)
            except Exception as exc:
                log.debug("WS send failed (%s) – marking dead.", exc)
                dead.append(ws)

        if dead:
            with self._ws_lock:
                for ws in dead:
                    event = self._ws_clients.pop(ws, None)
                    if event is not None:
                        event.set()

    @staticmethod
    def _send(ws, payload: dict) -> None:
        """Send a single JSON payload to one WebSocket."""
        try:
            ws.send(json.dumps(payload))
        except Exception as exc:
            log.debug("WS single send failed: %s", exc)

    # ── Snapshot / serialisation ──────────────────────────────────────────────

    def _public_snapshot(self) -> dict:
        """Build the JSON payload sent to clients."""
        snap = self._state.snapshot()
        players = [
            # JSON keys must be strings; convert int channel indices.
            {**p, "channels": {str(k): v for k, v in p["channels"].items()}}
            for p in snap["players"]
        ]
        return {
            "event":   "state_update",
            "players": players,
            # legacy mirrors of player 0
            "selected_vehicle": players[0]["selected_vehicle"],
            "channels":         players[0]["channels"],
        }

    def _find_vehicle(self, vehicle_id: str):
        for v in self._vehicles:
            if v.id == vehicle_id:
                return v
        return None

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self, host: str = "0.0.0.0", port: int = 5000, **kwargs) -> None:
        """Start the Flask development server (blocking)."""
        log.info("ControllerServer listening on %s:%d", host, port)
        self.app.run(host=host, port=port, **kwargs)

    def shutdown_ws_clients(self) -> None:
        """Signal all WebSocket handlers to close (for clean shutdown)."""
        with self._ws_lock:
            for event in self._ws_clients.values():
                event.set()


# ── Helpers ───────────────────────────────────────────────────────────────────

def _vehicle_dict(vehicle) -> dict:
    """Serialise a Vehicle dataclass or dict to a JSON-safe dict."""
    if isinstance(vehicle, dict):
        return vehicle
    return {
        "id":    vehicle.id,
        "name":  vehicle.name,
        "image": vehicle.image,
        "mac":   vehicle.mac,
    }
