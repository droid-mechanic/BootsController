# serial_bridge.py
"""
Phase 3 – Serial Bridge
Encodes channel state into fixed-length packets and writes them to the
ESP32 transmitter over USB serial.

Packet format (8 bytes):
    '<'  [ch0] [ch1] [ch2] [ch3] [ch4] [ch5]  '>'
         └─────────────────────────────────────┘
         6 channel bytes, each 0-255 (centre = 127)

Channels are indexed to match CHANNEL_MAP in state.py:
    0 = ABS_X      (Left Stick X)
    1 = ABS_Y      (Left Stick Y)
    2 = ABS_RX     (Right Stick X)
    3 = ABS_RY     (Right Stick Y)
    4 = ABS_HAT0X  (D-Pad X)
    5 = ABS_HAT0Y  (D-Pad Y)
"""

import logging
import time
import threading
import serial
import serial.serialutil

log = logging.getLogger(__name__)

# ── Constants ────────────────────────────────────────────────────────────────

NUM_CHANNELS    = 6
CENTRE_BYTE     = 127
PACKET_START    = b'<'
PACKET_END      = b'>'
PACKET_LEN      = 1 + NUM_CHANNELS + 1   # 8 bytes

DEFAULT_PORT     = "/dev/ttyUSB0"
DEFAULT_BAUD     = 115200
RECONNECT_DELAY  = 2.0   # seconds between reconnect attempts


# ── Encoding ─────────────────────────────────────────────────────────────────

def _encode_channel(value: float) -> int:
    """
    Map a normalised float in [-1.0, 1.0] to an unsigned byte [0, 255].
    Values outside the range are clamped.  Centre (0.0) → 127.
    """
    clamped = max(-1.0, min(1.0, value))
    return int(round((clamped + 1.0) * 0.5 * 254))   # 0 → 0, 0.0 → 127, 1.0 → 254


def build_packet(channels: dict[int, float]) -> bytes:
    """
    Build an 8-byte serial packet from a channel dict {index: float}.
    Missing channels default to centre (0.0).
    """
    payload = bytes(
        _encode_channel(channels.get(i, 0.0)) for i in range(NUM_CHANNELS)
    )
    return PACKET_START + payload + PACKET_END


# ── Serial bridge ─────────────────────────────────────────────────────────────

class SerialBridge:
    """
    Manages a serial connection to the ESP32 transmitter.

    Usage (context manager)::

        with SerialBridge() as bridge:
            bridge.write_channels(state.snapshot())

    Or manually::

        bridge = SerialBridge()
        bridge.open()
        bridge.write_channels(snapshot)
        bridge.close()

    ``write_channels`` is thread-safe; it can be called from the
    dispatcher thread while the main thread holds the port open.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
        self._port   = port
        self._baud   = baud
        self._serial: serial.Serial | None = None
        self._lock   = threading.Lock()

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def open(self) -> None:
        """Open the serial port, blocking until the connection succeeds."""
        while True:
            try:
                self._serial = serial.Serial(self._port, self._baud, timeout=1)
                log.info("Serial bridge open: %s @ %d baud", self._port, self._baud)
                return
            except serial.serialutil.SerialException as exc:
                log.warning("Serial open failed (%s) – retrying in %.1fs", exc, RECONNECT_DELAY)
                time.sleep(RECONNECT_DELAY)

    def close(self) -> None:
        """Close the serial port if open."""
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.close()
                log.info("Serial bridge closed.")
            self._serial = None

    def __enter__(self) -> "SerialBridge":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ── Write ────────────────────────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def write_channels(self, channels: dict[int, float]) -> bool:
        """
        Encode ``channels`` and write a packet to the serial port.

        ``channels`` is a dict mapping channel index → normalised float
        (matching the format returned by ``AppState.snapshot()['channels']``).

        Returns True on success, False if the port is not open or a write
        error occurs.  On error the port is closed so the caller can decide
        whether to reconnect.
        """
        packet = build_packet(channels)
        with self._lock:
            if not self.is_connected:
                log.debug("write_channels: port not open, dropping packet.")
                return False
            try:
                self._serial.write(packet)
                return True
            except serial.serialutil.SerialException as exc:
                log.error("Serial write error: %s – closing port.", exc)
                self._serial.close()
                self._serial = None
                return False

    def reconnect(self) -> None:
        """Close and reopen the serial port (blocking)."""
        self.close()
        self.open()
