/*
 * tx_firmware.ino
 * ═══════════════
 * Phase 8 — ESP32 Transmitter Firmware  (v2 — adds button bitmask)
 *
 * Role
 * ----
 * This ESP32 sits between the Raspberry Pi (USB serial) and the RC
 * vehicle fleet (ESP-NOW wireless).  It does three things:
 *
 *   1. Reads serial data from the Pi at 115 200 baud.
 *   2. Parses two command types:
 *        • Control packets  → 10-byte frame, forwarded via ESP-NOW
 *        • Peer commands    → ASCII line, updates the registered peer MAC
 *   3. Transmits control payloads to the registered peer via ESP-NOW.
 *
 *
 * Serial protocol (Pi → ESP32)
 * ----------------------------
 * CONTROL PACKET  (10 bytes, binary)
 *   Byte 0    : '<'  (0x3C) — start sentinel
 *   Bytes 1–6 : channel values, each 0–254 (centre = 127)
 *               ch0=ABS_X  ch1=ABS_Y  ch2=ABS_RX
 *               ch3=ABS_RY ch4=ABS_HAT0X ch5=ABS_HAT0Y
 *   Byte 7    : button bitmask 0  (bit0=A bit1=B bit2=X bit3=Y
 *                                  bit4=L1 bit5=R1 bit6=L2 bit7=R2)
 *   Byte 8    : button bitmask 1  (bit0=ThumbL bit1=ThumbR, bits 2-7 reserved)
 *   Byte 9    : '>'  (0x3E) — end sentinel
 *
 * PEER COMMAND  (ASCII, newline-terminated)
 *   "P aa:bb:cc:dd:ee:ff\n"   — register MAC as ESP-NOW peer
 *   "P 00:00:00:00:00:00\n"   — clear peer (stop transmitting)
 *
 *
 * ESP-NOW payload (ESP32 TX → ESP32 RX on vehicle)
 * -------------------------------------------------
 * struct ControlPayload {
 *     uint8_t ch[6];        // raw channel bytes, identical to serial packet
 *     uint8_t buttons[2];   // raw button bitmask bytes, identical to serial packet
 * };
 * 8 bytes.  The vehicle firmware interprets them however it likes.
 *
 *
 * Integration with app.py (Pi side)
 * ----------------------------------
 * When AppState.select(vehicle) fires, app.py should write:
 *
 *     bridge._serial.write(f"P {vehicle.mac}\n".encode())
 *
 * When AppState.deselect() fires:
 *
 *     bridge._serial.write(b"P 00:00:00:00:00:00\n")
 *
 * A convenience helper send_peer_command() in serial_bridge.py (Phase 3)
 * can wrap this — see the integration note at the bottom of this file.
 *
 * NOTE: gamepad.py / dispatcher.py need to be updated to read BTN_* evdev
 * codes and pack them into the two button bytes using the bit layout above
 * before writing the 10-byte control packet. That Pi-side change isn't
 * included here since gamepad.py/dispatcher.py weren't provided in this
 * session — happy to wire that up if you share them.
 *
 *
 * Board / library requirements
 * ----------------------------
 * Board   : ESP32 (Arduino-ESP32 core 2.x)
 * Library : esp_now.h, WiFi.h  (included in Arduino-ESP32 core)
 * Flash   : any ESP32 module with USB-CDC or CH340/CP210x USB-serial
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ── Serial config ─────────────────────────────────────────────────────────
static constexpr uint32_t SERIAL_BAUD      = 115200;
static constexpr uint8_t  PKT_START        = '<';
static constexpr uint8_t  PKT_END          = '>';
static constexpr uint8_t  NUM_CHANNELS     = 6;
static constexpr uint8_t  NUM_BUTTON_BYTES = 2;
static constexpr uint8_t  PKT_LEN          = 1 + NUM_CHANNELS + NUM_BUTTON_BYTES + 1;  // 10

// Maximum length of a peer command line incl. newline
// "P aa:bb:cc:dd:ee:ff\n" = 22 chars
static constexpr uint8_t  PEER_CMD_MAXLEN = 24;

// ── ESP-NOW payload ───────────────────────────────────────────────────────
struct __attribute__((packed)) ControlPayload {
    uint8_t ch[NUM_CHANNELS];
    uint8_t buttons[NUM_BUTTON_BYTES];
};

// ── State ─────────────────────────────────────────────────────────────────
static uint8_t    peerMac[6]   = {0};
static bool       hasPeer      = false;
static bool       peerAdded    = false;   // tracks esp_now_add_peer state

// Serial receive buffers
static uint8_t    binBuf[PKT_LEN];        // binary packet accumulator
static uint8_t    binIdx  = 0;
static bool       inPacket = false;

static char       lineBuf[PEER_CMD_MAXLEN];
static uint8_t    lineIdx = 0;

// ── Helpers ───────────────────────────────────────────────────────────────

/*
 * parseMac — parse "aa:bb:cc:dd:ee:ff" into a 6-byte array.
 * Returns true on success.
 */
static bool parseMac(const char* str, uint8_t* out) {
    // Expected: exactly 17 chars, colons at positions 2,5,8,11,14
    if (strlen(str) != 17) return false;
    for (int i = 0; i < 6; i++) {
        char high = str[i * 3];
        char low  = str[i * 3 + 1];
        char sep  = (i < 5) ? str[i * 3 + 2] : ':';  // last group has no sep
        if (i < 5 && sep != ':') return false;
        auto hexVal = [](char c) -> int8_t {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        int8_t h = hexVal(high), l = hexVal(low);
        if (h < 0 || l < 0) return false;
        out[i] = (uint8_t)((h << 4) | l);
    }
    return true;
}

static bool isZeroMac(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) if (mac[i] != 0) return false;
    return true;
}

/*
 * registerPeer — remove any existing peer and add the new one.
 */
static void registerPeer(const uint8_t* mac) {
    if (peerAdded) {
        esp_now_del_peer(peerMac);
        peerAdded = false;
    }

    memcpy(peerMac, mac, 6);
    hasPeer = !isZeroMac(mac);

    if (!hasPeer) {
        Serial.println("[TX] Peer cleared — transmit disabled.");
        return;
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, peerMac, 6);
    peer.channel = 0;       // 0 = current WiFi channel
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_OK) {
        peerAdded = true;
        Serial.printf("[TX] Peer set: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                      peerMac[0], peerMac[1], peerMac[2],
                      peerMac[3], peerMac[4], peerMac[5]);
    } else {
        Serial.printf("[TX] esp_now_add_peer error: %d\r\n", err);
        hasPeer = false;
    }
}

// ── ESP-NOW send callback (for diagnostics) ───────────────────────────────
static void onSent(const uint8_t* mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[TX] ESP-NOW send failed (no ACK).");
    }
}

// ── Packet processing ─────────────────────────────────────────────────────

/*
 * dispatchControl — validate and forward a complete 10-byte packet.
 * buf[0] == '<', buf[9] == '>', buf[1..6] are channel bytes,
 * buf[7..8] are button bitmask bytes.
 */
static void dispatchControl(const uint8_t* buf) {
    if (buf[0] != PKT_START || buf[PKT_LEN - 1] != PKT_END) {
        Serial.println("[TX] Malformed packet — sentinels invalid.");
        return;
    }

    if (!hasPeer) return;   // silently drop: no peer registered

    ControlPayload payload;
    memcpy(payload.ch, buf + 1, NUM_CHANNELS);
    memcpy(payload.buttons, buf + 1 + NUM_CHANNELS, NUM_BUTTON_BYTES);

    esp_err_t err = esp_now_send(peerMac, (uint8_t*)&payload, sizeof(payload));
    if (err != ESP_OK) {
        Serial.printf("[TX] esp_now_send error: %d\r\n", err);
    }
}

/*
 * handleLine — process a newline-terminated ASCII command.
 * Currently only the 'P' (peer) command is defined.
 */
static void handleLine(const char* line) {
    if (line[0] == 'P' && line[1] == ' ') {
        uint8_t mac[6];
        if (parseMac(line + 2, mac)) {
            registerPeer(mac);
        } else {
            Serial.printf("[TX] Bad MAC in peer command: '%s'\r\n", line + 2);
        }
    } else {
        Serial.printf("[TX] Unknown command: '%s'\r\n", line);
    }
}

// ── Arduino setup ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { /* wait for USB CDC on dev boards */ }

    Serial.println("[TX] RC Transmitter starting.");

    // ESP-NOW requires WiFi in station mode (no AP needed)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[TX] FATAL: esp_now_init() failed.");
        while (true) { delay(1000); }
    }

    esp_now_register_send_cb(onSent);

    Serial.printf("[TX] MAC: %s\r\n", WiFi.macAddress().c_str());
    Serial.println("[TX] Ready. Waiting for serial commands.");
}

// ── Arduino loop ──────────────────────────────────────────────────────────
void loop() {
    while (Serial.available()) {
        uint8_t byte = (uint8_t)Serial.read();

        // ── Binary packet path ──────────────────────────────────────────
        if (byte == PKT_START && !inPacket) {
            // Start of a new binary packet — reset and begin accumulating
            inPacket  = true;
            binIdx    = 0;
            binBuf[binIdx++] = byte;
            lineIdx   = 0;   // discard any partial ASCII line
            continue;
        }

        if (inPacket) {
            binBuf[binIdx++] = byte;

            if (binIdx == PKT_LEN) {
                // Full packet received
                inPacket = false;
                binIdx   = 0;
                dispatchControl(binBuf);
            } else if (binIdx > PKT_LEN) {
                // Overflow — something went wrong; reset
                Serial.println("[TX] Packet overrun — resync.");
                inPacket = false;
                binIdx   = 0;
            }
            continue;
        }

        // ── ASCII command path ──────────────────────────────────────────
        if (byte == '\n' || byte == '\r') {
            if (lineIdx > 0) {
                lineBuf[lineIdx] = '\0';
                handleLine(lineBuf);
                lineIdx = 0;
            }
        } else if (byte != PKT_START) {
            // Accumulate printable ASCII; ignore stray '<' in ASCII path
            // (a '<' always starts a binary packet instead — handled above)
            if (lineIdx < PEER_CMD_MAXLEN - 1) {
                lineBuf[lineIdx++] = (char)byte;
            } else {
                // Line overflow — discard
                Serial.println("[TX] ASCII line overflow — discarding.");
                lineIdx = 0;
            }
        }
    }
}


/*
 * ═══════════════════════════════════════════════════════════════════════════
 * INTEGRATION NOTE — Pi side (serial_bridge.py additions)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Add this method to SerialBridge in serial_bridge.py:
 *
 *     def send_peer_command(self, mac: str | None) -> bool:
 *         """
 *         Tell the ESP32 TX which peer MAC to target.
 *         Pass None or "00:00:00:00:00:00" to clear (stop transmitting).
 *         """
 *         target = mac if mac else "00:00:00:00:00:00"
 *         cmd = f"P {target}\n".encode()
 *         with self._lock:
 *             if not self.is_connected:
 *                 return False
 *             try:
 *                 self._serial.write(cmd)
 *                 return True
 *             except serial.serialutil.SerialException as exc:
 *                 log.error("send_peer_command error: %s", exc)
 *                 self._serial.close()
 *                 self._serial = None
 *                 return False
 *
 * Then in app.py, hook AppState.on_change to call it:
 *
 *     def on_state_change(reason: str) -> None:
 *         controller_server._on_state_change(reason)   # existing WS push
 *         snap = state.snapshot()
 *         vehicle = snap["selected_vehicle"]
 *         if reason in ("select", "deselect"):
 *             mac = vehicle.mac if vehicle else None
 *             bridge.send_peer_command(mac)
 *
 *     state.on_change = on_state_change
 *
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * BUTTON BITMASK — bit layout reference for the Pi side packer
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   buttons[0]  bit0=A  bit1=B  bit2=X  bit3=Y  bit4=L1  bit5=R1  bit6=L2  bit7=R2
 *   buttons[1]  bit0=ThumbL  bit1=ThumbR  (bits 2-7 reserved)
 *
 * dispatcher.py should build these two bytes from the evdev BTN_* key state
 * (BTN_SOUTH/BTN_EAST/BTN_NORTH/BTN_WEST, BTN_TL/BTN_TR/BTN_TL2/BTN_TR2,
 * BTN_THUMBL/BTN_THUMBR — see Evdev_Switch_classification.txt) and append
 * them to the 6 axis bytes before the '<' ... '>' frame is written.
 * ═══════════════════════════════════════════════════════════════════════════
 */
