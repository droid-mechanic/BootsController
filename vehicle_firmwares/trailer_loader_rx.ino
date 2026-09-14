/*
 * trailer_loader_rx.ino
 * ══════════════════════
 * Vehicle-side firmware — converted from the original Bluepad32 gamepad
 * sketch to receive its control inputs from the base-station transmitter
 * (tx_firmware.ino) over ESP-NOW instead of pairing directly to a
 * Bluetooth gamepad.
 *
 * All motor/servo/light/trailer logic below is unchanged from the
 * original sketch — only the *source* of the control values changed:
 * Bluepad32 ControllerPtr reads → ESP-NOW ControlPayload fields.
 *
 * ESP-NOW payload (from tx_firmware.ino)
 * ---------------------------------------
 *   struct ControlPayload {
 *       uint8_t ch[6];       // ch0=ABS_X ch1=ABS_Y ch2=ABS_RX
 *                             // ch3=ABS_RY ch4=ABS_HAT0X ch5=ABS_HAT0Y
 *                             // each byte 0-254, centre = 127
 *       uint8_t buttons[2];  // buttons[0]: bit0=A bit1=B bit2=X bit3=Y
 *                             //             bit4=L1 bit5=R1 bit6=L2 bit7=R2
 *                             // buttons[1]: bit0=ThumbL bit1=ThumbR
 *   };
 *   8 bytes total.
 *
 * Failsafe
 * --------
 * If no packet arrives for FAILSAFE_TIMEOUT_MS, all drive motors are
 * stopped and the trailer aux motors are commanded to STOP over the
 * daughter-board serial link, so a dropped wireless link doesn't leave
 * the vehicle running away uncontrolled.
 *
 * Board / library requirements
 * -----------------------------
 * Board   : ESP32 (Arduino-ESP32 core 2.x — matches tx_firmware.ino)
 * Library : ESP32Servo (by Kevin Harrington), esp_now.h, WiFi.h
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>

//25,26,32,33,21,19,22,23,2,4,17,16

/*What the different Serial commands for the trailer esp32 daughter board do
1-Trailer Legs Up
2-Trailer Legs Down
3-Ramp Up
4-Ramp Down
5-auxMotor1 Forward
6-auxMotor1 Reverse
7-auxMotor1 STOP
8-auxMotor2 Forward
9-auxMotor2 Reverse
10-auxMotor2 STOP
11- LT1 LOW
12- LT1 HIGH
13- LT2 LOW
14- LT2 HIGH
15- LT3 LOW
16- LT3 HIGH
*/
#define LT1 15
#define LT2 27
#define LT3 14

#define RX0 3
#define TX0 1

#define frontSteeringServoPin 23
#define hitchServoPin 22
#define clawServoPin 21

Servo frontSteeringServo;
Servo clawServo;

#define frontMotor0 33  // \ Used for controlling front drive motor movement
#define frontMotor1 32  // /
#define rearMotor0 2    // \ Used for controlling rear drive motor movement
#define rearMotor1 4    // /
#define rearMotor2 12   // \ Used for controlling second rear drive motor movement.
#define rearMotor3 13   // /

#define auxAttach0 17  // \ "AUX1" on PCB. Used for controlling auxillary motors or lights.
#define auxAttach1 16  // /
#define auxAttach2 5   // \ "Aux2" on PCB. Used for controlling auxillary motor or lights.  Keep in mind this will always breifly turn on when the model is powered on.
#define auxAttach3 18  // /
#define auxAttach4 25  // \ "Aux3" on PCB. Used for controlling auxillary motors or lights.
#define auxAttach5 26  // /


int lightSwitchButtonTime = 0;
int loaderModeButtonTime = 0;
int lightSwitchTime = 0;
int adjustedSteeringValue = 90;
int hitchServoValue = 160;
int steeringTrim = 0;
int lightMode = 0;
int clawServoValue = 90;
int servoDelay = 0;
bool lightsOn = false;
bool auxLightsOn = false;
bool blinkLT = false;
bool hazardLT = false;
bool hazardsOn = false;
bool moveClawServoUp = false;
bool moveClawServoDown = false;
bool loaderMode = false;
bool hitchUp = true;

bool trailerAuxMtr1Forward = false;
bool trailerAuxMtr1Reverse = false;
bool trailerAuxMtr2Forward = false;
bool trailerAuxMtr2Reverse = false;

// ── ESP-NOW control payload ────────────────────────────────────────────────
static constexpr uint8_t NUM_CHANNELS     = 6;
static constexpr uint8_t NUM_BUTTON_BYTES = 2;

struct __attribute__((packed)) ControlPayload {
    uint8_t ch[NUM_CHANNELS];
    uint8_t buttons[NUM_BUTTON_BYTES];
};

// Button bit layout (must match tx_firmware.ino / dispatcher.py)
#define BTN_A      (1 << 0)  // buttons[0]
#define BTN_B      (1 << 1)
#define BTN_X      (1 << 2)
#define BTN_Y      (1 << 3)
#define BTN_L1     (1 << 4)
#define BTN_R1     (1 << 5)
#define BTN_L2     (1 << 6)
#define BTN_R2     (1 << 7)
#define BTN_THUMBL (1 << 0)  // buttons[1]
#define BTN_THUMBR (1 << 1)  // buttons[1]

// Shared between the ESP-NOW recv callback (runs in the WiFi task) and loop()
static portMUX_TYPE payloadMux = portMUX_INITIALIZER_UNLOCKED;
static ControlPayload latestPayload = {};
static volatile bool  packetFresh   = false;
static volatile uint32_t lastPacketMillis = 0;

static constexpr uint32_t FAILSAFE_TIMEOUT_MS = 500;
static bool failsafeActive = false;

// ── Helpers: convert raw channel/button bytes into original control shapes ─

/*
 * byteToAxis — convert a 0-254 (centre=127) channel byte into roughly the
 * same numeric range the original Bluepad32 axis functions returned
 * (~-508..508), so the existing threshold-based logic below (comparisons
 * against 80, 90, 110, 200, etc.) keeps working unmodified.
 */
static inline int16_t byteToAxis(uint8_t b) {
    int16_t centered = (int16_t)b - 127;  // -127..127
    return centered * 4;                   // ~-508..508
}

/*
 * decodeDpad — reconstruct the same single-direction dpad value the
 * original Bluepad32 ctl->dpad() bitmask produced (1=UP 2=DOWN 4=RIGHT
 * 8=LEFT, 0=centered) from the HAT0X/HAT0Y axis bytes.
 */
static inline int decodeDpad(uint8_t hatXByte, uint8_t hatYByte) {
    static constexpr int16_t DPAD_THRESHOLD = 40;  // deadzone around centre (127)
    int16_t x = (int16_t)hatXByte - 127;
    int16_t y = (int16_t)hatYByte - 127;

    if (y < -DPAD_THRESHOLD) return 1;  // UP
    if (y >  DPAD_THRESHOLD) return 2;  // DOWN
    if (x >  DPAD_THRESHOLD) return 4;  // RIGHT
    if (x < -DPAD_THRESHOLD) return 8;  // LEFT
    return 0;
}

// ── ESP-NOW receive callback ────────────────────────────────────────────────
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len != sizeof(ControlPayload)) return;  // ignore malformed/foreign packets

    portENTER_CRITICAL(&payloadMux);
    memcpy(&latestPayload, incomingData, sizeof(ControlPayload));
    packetFresh = true;
    lastPacketMillis = millis();
    portEXIT_CRITICAL(&payloadMux);
}

// ── Failsafe: stop everything if the link drops ─────────────────────────────
void applyFailsafe() {
    moveMotor(rearMotor2, rearMotor3, 0);
    moveMotor(frontMotor0, frontMotor1, 0);
    moveMotor(auxAttach2, auxAttach3, 0);
    moveMotor(rearMotor0, rearMotor1, 0);
    moveMotor(auxAttach0, auxAttach1, 0);
    moveMotor(auxAttach5, auxAttach4, 0);

    // Stop trailer daughter-board aux motors too
    Serial.println(7);
    delay(10);
    Serial.println(10);
}

void processGamepad(const ControlPayload &p) {
    bool btnA = p.buttons[0] & BTN_A;
    bool btnB = p.buttons[0] & BTN_B;
    bool btnX = p.buttons[0] & BTN_X;
    bool btnY = p.buttons[0] & BTN_Y;
    bool btnL1 = p.buttons[0] & BTN_L1;
    bool btnR1 = p.buttons[0] & BTN_R1;
    bool btnL2 = p.buttons[0] & BTN_L2;
    bool btnR2 = p.buttons[0] & BTN_R2;
    bool btnThumbL = p.buttons[1] & BTN_THUMBL;
    bool btnThumbR = p.buttons[1] & BTN_THUMBR;

    int axisX  = byteToAxis(p.ch[0]);
    int axisY  = byteToAxis(p.ch[1]);
    int axisRX = byteToAxis(p.ch[2]);
    int axisRY = byteToAxis(p.ch[3]);
    int dpadValue = decodeDpad(p.ch[4], p.ch[5]);

    //Throttle
    processThrottleDipper(axisY);
    //Steering
    processSteeringGrapple(axisRX);
    //Steering trim and hitch
    processTrimAndHitch(dpadValue);
    //Lights
    processLights(btnThumbR);
    processLoaderMode(btnThumbL);

    //TrailerServos
    if (!loaderMode) {
        processTrailerLegsUp(btnY ? 1 : 0);
        processTrailerLegsDown(btnA ? 1 : 0);
    }
    processTrailerRampUp(btnB ? 1 : 0);
    processTrailerRampDown(btnX ? 1 : 0);
    //TrailerMTRs
    processTrailerAuxMtr1Forward(btnR2);
    processTrailerAuxMtr1Reverse(btnR1);
    processTrailerAuxMtr2Forward(btnL2);
    processTrailerAuxMtr2Reverse(btnL1);

    //Loader
    processPivot(axisX);
    processBoom(axisRY);

    if (blinkLT && (millis() - lightSwitchTime) > 300) {
        if (!lightsOn) {
            if (adjustedSteeringValue <= 70) {
                digitalWrite(LT1, HIGH);
                Serial.println(12);
            } else if (adjustedSteeringValue >= 110) {
                digitalWrite(LT2, HIGH);
                Serial.println(14);
            }
            lightsOn = true;
        } else {
            if (adjustedSteeringValue <= 70) {
                digitalWrite(LT2, HIGH);
                digitalWrite(LT1, LOW);
                Serial.println(11);
                delay(10);
                Serial.println(14);
            } else if (adjustedSteeringValue >= 110) {
                digitalWrite(LT1, HIGH);
                digitalWrite(LT2, LOW);
                Serial.println(13);
                delay(10);
                Serial.println(12);
            }
            lightsOn = false;
        }
        lightSwitchTime = millis();
    }
    if (blinkLT && adjustedSteeringValue > 70 && adjustedSteeringValue < 110) {
        digitalWrite(LT1, HIGH);
        digitalWrite(LT2, HIGH);
        Serial.println(12);
        delay(10);
        Serial.println(14);
    }
    if (hazardLT && (millis() - lightSwitchTime) > 300) {
        if (!hazardsOn) {
            digitalWrite(LT1, HIGH);
            digitalWrite(LT2, HIGH);
            Serial.println(12);
            delay(10);
            Serial.println(14);
            hazardsOn = true;
        } else {
            digitalWrite(LT1, LOW);
            digitalWrite(LT2, LOW);
            Serial.println(11);
            delay(10);
            Serial.println(13);
            hazardsOn = false;
        }
        lightSwitchTime = millis();
    }
    if (btnA) {
        moveClawServoDown = true;
    } else if (btnY) {
        moveClawServoUp = true;
    } else {
        moveClawServoDown = false;
        moveClawServoUp = false;
    }
    if (loaderMode) {
        if (moveClawServoUp) {
            if (servoDelay == 2) {
                if (clawServoValue >= 10 && clawServoValue < 170) {
                    clawServoValue = clawServoValue + 2;
                    clawServo.write(clawServoValue);
                }
                servoDelay = 0;
            }
            servoDelay++;
        }
        if (moveClawServoDown) {
            if (servoDelay == 2) {
                if (clawServoValue <= 170 && clawServoValue > 10) {
                    clawServoValue = clawServoValue - 2;
                    clawServo.write(clawServoValue);
                }
                servoDelay = 0;
            }
            servoDelay++;
        }
    }
}
void processTrailerLegsUp(int value) {
    if (value == 1) {
        Serial.println(1);
        delay(10);
    }
}
void processTrailerLegsDown(int value) {
    if (value == 1) {
        Serial.println(2);
        delay(10);
    }
}
void processTrailerRampUp(int value) {
    if (value == 1) {
        Serial.println(3);
        delay(10);
    }
}
void processTrailerRampDown(int value) {
    if (value == 1) {
        Serial.println(4);
        delay(10);
    }
}
void processPivot(int axisXValue) {
    int adjustedThrottleValue = axisXValue / 2;
    if (loaderMode && (adjustedThrottleValue > 90 || adjustedThrottleValue < -90)) {
        moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue - 90);
    }
    if (loaderMode) {
        if (adjustedThrottleValue > 90 && adjustedThrottleValue < 200) {
            moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue - 90);
        } else if (adjustedThrottleValue > 200) {
            moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue);
        } else if (adjustedThrottleValue < -90 && adjustedThrottleValue > -200) {
            moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue + 90);
        } else if (adjustedThrottleValue < -200) {
            moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue);
        } else if (adjustedThrottleValue > -90 && adjustedThrottleValue < 90) {
            moveMotor(rearMotor0, rearMotor1, 0);
        }
    }
}
void processBoom(int axisRYValue) {
    int adjustedThrottleValue = axisRYValue / 2;
    if (loaderMode) {
        if (adjustedThrottleValue > 80 || adjustedThrottleValue < -80) {
            moveMotor(auxAttach2, auxAttach3, adjustedThrottleValue);
        } else if (adjustedThrottleValue > -80 && adjustedThrottleValue < 80) {
            moveMotor(auxAttach2, auxAttach3, 0);
        }
    }
}
void processThrottleDipper(int axisYValue) {
    int adjustedThrottleValue = axisYValue / 2;
    if (!loaderMode) {
        int smokeThrottle = adjustedThrottleValue / 3;
        moveMotor(rearMotor2, rearMotor3, adjustedThrottleValue);
        moveMotor(frontMotor0, frontMotor1, adjustedThrottleValue);
        moveMotor(auxAttach2, auxAttach3, smokeThrottle);
    } else if (adjustedThrottleValue > 80 || adjustedThrottleValue < -80) {
        moveMotor(auxAttach0, auxAttach1, adjustedThrottleValue);
    } else if (adjustedThrottleValue > -80 && adjustedThrottleValue < 80) {
        moveMotor(auxAttach0, auxAttach1, 0);
    }
}

void processTrimAndHitch(int dpadValue) {
    if (dpadValue == 4 && steeringTrim < 20) {
        steeringTrim = steeringTrim + 1;
        delay(50);
    } else if (dpadValue == 8 && steeringTrim > -20) {
        steeringTrim = steeringTrim - 1;
        delay(50);
    }
    if (dpadValue == 1 || dpadValue == 2) {
        clawServo.detach();
        delay(50);
        clawServo.attach(hitchServoPin);
        if (dpadValue == 1) {
            clawServo.write(hitchServoValue);
            delay(40);
        } else if (dpadValue == 2) {
            clawServo.write(85);
            delay(40);
        }
        clawServo.detach();
        delay(20);
        clawServo.attach(clawServoPin);
    }
}
void processSteeringGrapple(int axisRXValue) {
    int adjustedThrottleValue = axisRXValue / 2;
    if (!loaderMode) {
        adjustedSteeringValue = (90 - (axisRXValue / 9)) - steeringTrim;
        frontSteeringServo.write(180 - adjustedSteeringValue);

        Serial.print("Steering Value:");
        Serial.println(adjustedSteeringValue);
    } else if (adjustedThrottleValue > 90 || adjustedThrottleValue < -90) {
        moveMotor(auxAttach5, auxAttach4, adjustedThrottleValue);
    } else if (adjustedThrottleValue > -110 && adjustedThrottleValue < 110) {
        moveMotor(auxAttach5, auxAttach4, 0);
    }
}

void processLights(bool buttonValue) {
    if (buttonValue && (millis() - lightSwitchButtonTime) > 300) {
        lightMode++;
        if (lightMode == 1) {
            digitalWrite(LT1, HIGH);
            digitalWrite(LT2, HIGH);
            Serial.println(12);
            delay(10);
            Serial.println(14);
        } else if (lightMode == 2) {
            digitalWrite(LT1, LOW);
            digitalWrite(LT2, LOW);
            delay(100);
            digitalWrite(LT1, HIGH);
            digitalWrite(LT2, HIGH);
            blinkLT = true;
        } else if (lightMode == 3) {
            blinkLT = false;
            hazardLT = true;
        } else if (lightMode == 4) {
            hazardLT = false;
            digitalWrite(LT1, LOW);
            digitalWrite(LT2, LOW);
            Serial.println(11);
            delay(10);
            Serial.println(13);
            lightMode = 0;
            if (!auxLightsOn) {
                digitalWrite(LT3, HIGH);
                Serial.println(16);
                auxLightsOn = true;
            } else {
                digitalWrite(LT3, LOW);
                Serial.println(15);
                auxLightsOn = false;
            }
        }
        lightSwitchButtonTime = millis();
    }
}
void processTrailerAuxMtr1Forward(bool value) {
    if (value) {
        Serial.println(5);
        delay(10);
        trailerAuxMtr1Forward = true;
    } else if (trailerAuxMtr1Forward) {
        Serial.println(7);
        delay(10);
        trailerAuxMtr1Forward = false;
    }
}
void processTrailerAuxMtr1Reverse(bool value) {
    if (value) {
        Serial.println(6);
        delay(10);
        trailerAuxMtr1Reverse = true;
    } else if (trailerAuxMtr1Reverse) {
        Serial.println(7);
        delay(10);
        trailerAuxMtr1Reverse = false;
    }
}
void processTrailerAuxMtr2Forward(bool value) {
    if (value) {
        Serial.println(8);
        delay(10);
        trailerAuxMtr2Forward = true;
    } else if (trailerAuxMtr2Forward) {
        Serial.println(10);
        delay(10);
        trailerAuxMtr2Forward = false;
    }
}
void processTrailerAuxMtr2Reverse(bool value) {
    if (value) {
        Serial.println(9);
        delay(10);
        trailerAuxMtr2Reverse = true;
    } else if (trailerAuxMtr2Reverse) {
        Serial.println(10);
        delay(10);
        trailerAuxMtr2Reverse = false;
    }
}
void processLoaderMode(bool buttonValue) {
    if (buttonValue && (millis() - loaderModeButtonTime) > 300) {
        if (!loaderMode) {
            loaderMode = true;
            moveMotor(rearMotor2, rearMotor3, 0);
            moveMotor(frontMotor0, frontMotor1, 0);
            moveMotor(auxAttach2, auxAttach3, 0);
        } else {
            loaderMode = false;
        }
        loaderModeButtonTime = millis();
    }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
    if (velocity > 15) {
        analogWrite(motorPin0, velocity);
        analogWrite(motorPin1, LOW);
    } else if (velocity < -15) {
        analogWrite(motorPin0, LOW);
        analogWrite(motorPin1, (-1 * velocity));
    } else {
        analogWrite(motorPin0, 0);
        analogWrite(motorPin1, 0);
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.println("[RX] Vehicle firmware starting (ESP-NOW mode).");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[RX] FATAL: esp_now_init() failed.");
        while (true) { delay(1000); }
    }
    esp_now_register_recv_cb(onDataRecv);

    Serial.printf("[RX] MAC: %s\r\n", WiFi.macAddress().c_str());
    Serial.println("[RX] Ready. Waiting for ESP-NOW control packets.");

    pinMode(rearMotor0, OUTPUT);
    pinMode(rearMotor1, OUTPUT);
    pinMode(rearMotor2, OUTPUT);
    pinMode(rearMotor3, OUTPUT);
    pinMode(frontMotor0, OUTPUT);
    pinMode(frontMotor1, OUTPUT);
    pinMode(auxAttach0, OUTPUT);
    pinMode(auxAttach1, OUTPUT);
    pinMode(auxAttach2, OUTPUT);
    pinMode(auxAttach3, OUTPUT);
    pinMode(auxAttach4, OUTPUT);
    pinMode(auxAttach5, OUTPUT);
    pinMode(LT1, OUTPUT);
    pinMode(LT2, OUTPUT);
    pinMode(LT3, OUTPUT);

    digitalWrite(rearMotor0, LOW);
    digitalWrite(rearMotor1, LOW);
    digitalWrite(rearMotor2, LOW);
    digitalWrite(rearMotor3, LOW);
    digitalWrite(frontMotor0, LOW);
    digitalWrite(frontMotor1, LOW);
    digitalWrite(auxAttach0, LOW);
    digitalWrite(auxAttach1, LOW);
    digitalWrite(auxAttach2, LOW);
    digitalWrite(auxAttach3, LOW);
    digitalWrite(auxAttach4, LOW);
    digitalWrite(auxAttach5, LOW);
    digitalWrite(LT1, LOW);
    digitalWrite(LT2, LOW);
    digitalWrite(LT3, LOW);

    frontSteeringServo.attach(frontSteeringServoPin);
    frontSteeringServo.write(adjustedSteeringValue);
    clawServo.attach(clawServoPin);
    clawServo.write(clawServoValue);

    lastPacketMillis = millis();  // don't trip the failsafe before the first packet
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    bool timedOut = (millis() - lastPacketMillis) > FAILSAFE_TIMEOUT_MS;

    if (timedOut) {
        if (!failsafeActive) {
            Serial.println("[RX] Link lost — applying failsafe stop.");
            applyFailsafe();
            failsafeActive = true;
        }
    } else if (packetFresh) {
        ControlPayload snapshot;
        portENTER_CRITICAL(&payloadMux);
        snapshot = latestPayload;
        packetFresh = false;
        portEXIT_CRITICAL(&payloadMux);

        failsafeActive = false;
        processGamepad(snapshot);
    }

    // Must yield periodically or the watchdog will trigger.
    vTaskDelay(1);
}
