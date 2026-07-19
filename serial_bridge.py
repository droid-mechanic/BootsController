# serial_bridge.py
"""
Phase 3 – Serial Bridge
Encodes channel + button state into fixed-length packets and writes them
to the ESP32 transmitter over USB serial. Also sends peer-MAC commands
so the base station knows which vehicle to target.

Packet format (10 bytes):
    '<'  [ch0] [ch1] [ch2] [ch3] [ch4] [ch5]  [btn0] [btn1]  '>'
         └───────────────────────────────────┘└─────────────┘
         6 channel bytes, each 0-254 (centre=127)   2 button bitmask bytes

Channels are indexed to match CHANNEL_MAP in state.py:
    0 = ABS_X      (Left Stick X)
    1 = ABS_Y      (Left Stick Y)
    2 = ABS_RX     (Right Stick X)
    3 = ABS_RY     (Right Stick Y)
    4 = ABS_HAT0X  (D-Pad X)
    5 = ABS_HAT0Y  (D-Pad Y)

Buttons are a 2-byte bitmask matching dispatcher.py's BUTTON_BIT_MAP /
tx_firmware.ino's ControlPayload.buttons[]:
    buttons[0]  bit0=A  bit1=B  bit2=X  bit3=Y  bit4=L1  bit5=R1  bit6=L2  bit7=R2
    buttons[1]  bit0=ThumbL  bit1=ThumbR  (bits 2-7 reserved)

Peer command (ASCII, newline-terminated):
    "P aa:bb:cc:dd:ee:ff\\n"   — tell the ESP32 TX which vehicle MAC to target
    "P 00:00:00:00:00:00\\n"   — clear peer (stop transmitting)
"""

import logging
import time
import threading
import serial
import serial.serialutil

log = logging.getLogger(__name__)

# ── Constants ────────────────────────────────────────────────────────────────

NUM_CHANNELS     = 6
NUM_BUTTON_BYTES = 2
CENTRE_BYTE      = 127
PACKET_START     = b'<'
PACKET_END       = b'>'
PACKET_LEN       = 1 + NUM_CHANNELS + NUM_BUTTON_BYTES + 1   # 10 bytes

DEFAULT_PORT     = "/dev/ttyUSB0"
DEFAULT_BAUD     = 115200
RECONNECT_DELAY  = 2.0   # seconds between reconnect attempts

NULL_MAC = "00:00:00:00:00:00"


# ── Encoding ─────────────────────────────────────────────────────────────────

def _encode_channel(value: float) -> int:
    """
    Map a normalised float in [-1.0, 1.0] to an unsigned byte [0, 254].
    Values outside the range are clamped.  Centre (0.0) → 127.
    """
    clamped = max(-1.0, min(1.0, value))
    return int(round((clamped + 1.0) * 0.5 * 254))   # -1.0 → 0, 0.0 → 127, 1.0 → 254


def build_packet(channels: dict[int, float], buttons: bytes = b"\x00\x00") -> bytes:
    """
    Build a 10-byte serial packet from a channel dict {index: float} and
    a 2-byte button bitmask.  Missing channels default to centre (0.0).
    """
    if len(buttons) != NUM_BUTTON_BYTES:
        raise ValueError(f"buttons must be {NUM_BUTTON_BYTES} bytes, got {len(buttons)}")

    payload = bytes(
        _encode_channel(channels.get(i, 0.0)) for i in range(NUM_CHANNELS)
    )
    return PACKET_START + payload + bytes(buttons) + PACKET_END


# ── Serial bridge ─────────────────────────────────────────────────────────────

class SerialBridge:
    """
    Manages a serial connection to the ESP32 transmitter.

    Usage (context manager)::

        with SerialBridge() as bridge:
            bridge.write_channels(channels, buttons)

    Or manually::

        bridge = SerialBridge()
        bridge.open()
        bridge.write_channels(channels, buttons)
        bridge.close()

    ``write_channels`` and ``send_peer_command`` are thread-safe; they can
    be called from the dispatcher thread while the main thread holds the
    port open.
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

    def write_channels(self, channels: dict[int, float], buttons: bytes = b"\x00\x00") -> bool:
        """
        Encode ``channels`` + ``buttons`` and write a packet to the serial port.

        ``channels`` is a dict mapping channel index → normalised float
        (matching the format returned by ``AppState.snapshot()['channels']``).
        ``buttons`` is a 2-byte bitmask (see module docstring / dispatcher.py's
        BUTTON_BIT_MAP for the bit layout).

        Returns True on success, False if the port is not open or a write
        error occurs.  On error the port is closed so the caller can decide
        whether to reconnect.
        """
        packet = build_packet(channels, buttons)
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

    def send_peer_command(self, mac: str | None) -> bool:
        """
        Tell the ESP32 TX which vehicle MAC to target for ESP-NOW.

        Pass ``None`` (or the null MAC) to clear the peer and stop
        transmitting — call this on vehicle deselect.

        Returns True on success, False if the port is not open or a write
        error occurs.
        """
        target = mac if mac else NULL_MAC
        cmd = f"P {target}\n".encode("ascii")

        with self._lock:
            if not self.is_connected:
                log.debug("send_peer_command: port not open, dropping command.")
                return False
            try:
                self._serial.write(cmd)
                log.info("Peer command sent: %s", target)
                return True
            except serial.serialutil.SerialException as exc:
                log.error("send_peer_command error: %s – closing port.", exc)
                self._serial.close()
                self._serial = None
                return False

    def reconnect(self) -> None:
        """Close and reopen the serial port (blocking)."""
        self.close()
        self.open()
