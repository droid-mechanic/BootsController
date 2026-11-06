/*
 * tx_firmware.ino
 * ═══════════════
 * ESP32 Transmitter Firmware  (v3 - two player slots)
 *
 * Role
 * ----
 * This ESP32 sits between the Raspberry Pi (USB serial) and the RC
 * vehicle fleet (ESP-NOW wireless).  It does three things:
 *
 *   1. Reads serial data from the Pi at 115 200 baud.
 *   2. Parses two command types:
 *        • Control packets  → 11-byte frame naming a slot, forwarded via ESP-NOW
 *        • Peer commands    → ASCII line, sets the vehicle MAC for a slot
 *   3. Transmits each control payload to the vehicle registered for its slot.
 *
 * A "slot" is one operator (0 or 1). Each slot targets its own vehicle, so
 * two operators can drive two different vehicles at the same time.
 *
 *
 * Serial protocol (Pi → ESP32)
 * ----------------------------
 * CONTROL PACKET  (11 bytes, binary)
 *   Byte 0     : '<'  (0x3C) - start sentinel
 *   Byte 1     : slot, 0 or 1
 *   Bytes 2–7  : channel values, each 0–254 (centre = 127)
 *                ch0=ABS_X  ch1=ABS_Y  ch2=ABS_RX
 *                ch3=ABS_RY ch4=ABS_HAT0X ch5=ABS_HAT0Y
 *   Byte 8     : button bitmask 0  (bit0=A bit1=B bit2=X bit3=Y
 *                                   bit4=L1 bit5=R1 bit6=L2 bit7=R2)
 *   Byte 9     : button bitmask 1  (bit0=ThumbL bit1=ThumbR, bits 2-7 reserved)
 *   Byte 10    : '>'  (0x3E) - end sentinel
 *
 * PEER COMMAND  (ASCII, newline-terminated)
 *   "P <slot> aa:bb:cc:dd:ee:ff\n"   - slot targets this vehicle MAC
 *   "P <slot> 00:00:00:00:00:00\n"   - clear the slot (stop transmitting)
 *
 * Peer commands are idempotent: re-sending the same MAC is a no-op. The Pi
 * re-sends them every couple of seconds, so a transmitter reset recovers
 * on its own.
 *
 *
 * ESP-NOW payload (ESP32 TX → ESP32 RX on vehicle)
 * -------------------------------------------------
 * struct ControlPayload {
 *     uint8_t ch[6];        // raw channel bytes, identical to serial packet
 *     uint8_t buttons[2];   // raw button bitmask bytes, identical to serial packet
 * };
 * 8 bytes - unchanged, so vehicle firmware needs no modification. Vehicles
 * are addressed by destination MAC; the slot never goes over the air.
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
static constexpr uint8_t  NUM_SLOTS        = 2;   // one per operator
static constexpr uint8_t  PKT_LEN          = 1 + 1 + NUM_CHANNELS + NUM_BUTTON_BYTES + 1;  // 11

// Maximum length of a peer command line incl. CR/LF
// "P 0 aa:bb:cc:dd:ee:ff\n" = 22 chars
static constexpr uint8_t  PEER_CMD_MAXLEN = 28;

// ── ESP-NOW payload ───────────────────────────────────────────────────────
struct __attribute__((packed)) ControlPayload {
    uint8_t ch[NUM_CHANNELS];
    uint8_t buttons[NUM_BUTTON_BYTES];
};

// ── State ─────────────────────────────────────────────────────────────────
static uint8_t    peerMac[NUM_SLOTS][6] = {};
static bool       hasPeer[NUM_SLOTS]    = {};   // slot has a registered ESP-NOW peer

// Serial receive buffers
static uint8_t    binBuf[PKT_LEN];        // binary packet accumulator
static uint8_t    binIdx  = 0;
static bool       inPacket = false;

static char       lineBuf[PEER_CMD_MAXLEN];
static uint8_t    lineIdx = 0;

// ── Helpers ───────────────────────────────────────────────────────────────

/*
 * parseMac - parse "aa:bb:cc:dd:ee:ff" into a 6-byte array.
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
 * macInUseByOther - true if another slot already targets this MAC.
 */
static bool macInUseByOther(uint8_t slot, const uint8_t* mac) {
    for (uint8_t s = 0; s < NUM_SLOTS; s++) {
        if (s != slot && hasPeer[s] && memcmp(peerMac[s], mac, 6) == 0) return true;
    }
    return false;
}

/*
 * registerPeer - point a slot at a vehicle MAC (all zeros clears the slot).
 * Idempotent: the Pi re-sends peers periodically, and an unchanged request
 * does nothing.
 */
static void registerPeer(uint8_t slot, const uint8_t* mac) {
    const bool want = !isZeroMac(mac);

    if (want == hasPeer[slot] && (!want || memcmp(peerMac[slot], mac, 6) == 0)) {
        return;   // no change
    }

    // Release this slot's previous vehicle unless another slot still uses it.
    if (hasPeer[slot] && !macInUseByOther(slot, peerMac[slot])) {
        esp_now_del_peer(peerMac[slot]);
    }
    hasPeer[slot] = false;

    if (!want) {
        memset(peerMac[slot], 0, 6);
        Serial.printf("[TX] Slot %u cleared - transmit disabled.\r\n", slot);
        return;
    }

    memcpy(peerMac[slot], mac, 6);

    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac, 6);
        peer.channel = 0;       // 0 = current WiFi channel
        peer.encrypt = false;

        esp_err_t err = esp_now_add_peer(&peer);
        if (err != ESP_OK) {
            Serial.printf("[TX] esp_now_add_peer error: %d\r\n", err);
            return;             // hasPeer stays false; the next refresh retries
        }
    }

    hasPeer[slot] = true;
    Serial.printf("[TX] Slot %u peer set: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                  slot, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ── ESP-NOW send callback (for diagnostics) ───────────────────────────────
static void onSent(const uint8_t* mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[TX] ESP-NOW send failed (no ACK).");
    }
}

// ── Packet processing ─────────────────────────────────────────────────────

/*
 * dispatchControl - validate and forward a complete 11-byte packet.
 * buf[0] == '<', buf[1] = slot, buf[2..7] = channel bytes,
 * buf[8..9] = button bitmask bytes, buf[10] == '>'.
 */
static void dispatchControl(const uint8_t* buf) {
    if (buf[0] != PKT_START || buf[PKT_LEN - 1] != PKT_END) {
        Serial.println("[TX] Malformed packet - sentinels invalid.");
        return;
    }

    const uint8_t slot = buf[1];
    if (slot >= NUM_SLOTS) {
        Serial.printf("[TX] Bad slot in control packet: %u\r\n", slot);
        return;
    }

    if (!hasPeer[slot]) return;   // silently drop: no vehicle for this slot

    ControlPayload payload;
    memcpy(payload.ch, buf + 2, NUM_CHANNELS);
    memcpy(payload.buttons, buf + 2 + NUM_CHANNELS, NUM_BUTTON_BYTES);

    esp_err_t err = esp_now_send(peerMac[slot], (uint8_t*)&payload, sizeof(payload));
    if (err != ESP_OK) {
        Serial.printf("[TX] esp_now_send error (slot %u): %d\r\n", slot, err);
    }
}

/*
 * handleLine - process a newline-terminated ASCII command.
 * Currently only the 'P' (peer) command is defined:  "P <slot> <mac>"
 */
static void handleLine(const char* line) {
    if (line[0] == 'P' && line[1] == ' ' &&
        line[2] >= '0' && line[2] < ('0' + NUM_SLOTS) && line[3] == ' ') {
        const uint8_t slot = (uint8_t)(line[2] - '0');
        uint8_t mac[6];
        if (parseMac(line + 4, mac)) {
            registerPeer(slot, mac);
        } else {
            Serial.printf("[TX] Bad MAC in peer command: '%s'\r\n", line + 4);
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
            // Start of a new binary packet - reset and begin accumulating
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
                // Overflow - something went wrong; reset
                Serial.println("[TX] Packet overrun - resync.");
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
            // (a '<' always starts a binary packet instead - handled above)
            if (lineIdx < PEER_CMD_MAXLEN - 1) {
                lineBuf[lineIdx++] = (char)byte;
            } else {
                // Line overflow - discard
                Serial.println("[TX] ASCII line overflow - discarding.");
                lineIdx = 0;
            }
        }
    }
}
