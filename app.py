# app.py
"""
Main entry point – boots the full controller pipeline and HTTP server.

Start with:
    python app.py

Environment / tunables (edit directly or move to a config file later):
    GAMEPAD_PATH   – evdev device path   (default: /dev/input/event0)
    SERIAL_PORT    – USB serial port     (default: /dev/ttyUSB0)
    SERIAL_BAUD    – baud rate           (default: 115200)
    SERVER_HOST    – Flask bind address  (default: 0.0.0.0)
    SERVER_PORT    – Flask port          (default: 5000)
"""

import logging
import os
import signal
import sys

# ── Logging (configure before importing project modules) ──────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("app")

# ── Project imports ───────────────────────────────────────────────────────────
from gamepad       import GamepadReader
from state         import AppState, load_vehicles
from serial_bridge import SerialBridge
from dispatcher    import Dispatcher
from server        import ControllerServer

# ── Config ────────────────────────────────────────────────────────────────────
GAMEPAD_PATH = os.getenv("GAMEPAD_PATH", "/dev/input/event5")
SERIAL_PORT  = os.getenv("SERIAL_PORT",  "/dev/ttyUSB0")
SERIAL_BAUD  = int(os.getenv("SERIAL_BAUD", "115200"))
SERVER_HOST  = os.getenv("SERVER_HOST",  "0.0.0.0")
SERVER_PORT  = int(os.getenv("SERVER_PORT", "5000"))
VEHICLES_FILE = "vehicles.json"


def main() -> None:
    log.info("=== RC Controller starting ===")

    # ── Load vehicles ─────────────────────────────────────────────────────────
    vehicles = load_vehicles(VEHICLES_FILE)
    log.info("Loaded %d vehicle(s) from %s", len(vehicles), VEHICLES_FILE)

    # ── Shared state ──────────────────────────────────────────────────────────
    state = AppState()

    # ── Gamepad ───────────────────────────────────────────────────────────────
    reader = GamepadReader(GAMEPAD_PATH)
    reader.start()

    # ── Serial bridge ─────────────────────────────────────────────────────────
    bridge = SerialBridge(port=SERIAL_PORT, baud=SERIAL_BAUD)
    bridge.open()   # blocks until port is available

    # ── Dispatcher ────────────────────────────────────────────────────────────
    dispatcher = Dispatcher(reader, state, bridge)
    dispatcher.start()
    log.info("Dispatcher running: %s", dispatcher.is_running)
    log.info("Reader queue id: %s", id(reader.queue))

    # ── HTTP / WebSocket server ───────────────────────────────────────────────
    controller_server = ControllerServer(
        state=state,
        reader=reader,
        bridge=bridge,
        dispatcher=dispatcher,
        vehicles=vehicles,
    )

    # ── Graceful shutdown on SIGINT / SIGTERM ─────────────────────────────────
    def _shutdown(signum, frame):
        log.info("Shutdown signal received — stopping subsystems.")
        controller_server.shutdown_ws_clients()
        dispatcher.stop()
        bridge.close()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── Blocking: run Flask (main thread) ─────────────────────────────────────
    controller_server.run(
        host=SERVER_HOST,
        port=SERVER_PORT,
        use_reloader=False,   # reloader forks and breaks evdev/serial threads
        threaded=True,        # needed for concurrent WebSocket connections
    )


if __name__ == "__main__":
    main()
