#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>

// ============================================================================
// AtomRC gimbal RX bridge (drone side)
// ----------------------------------------------------------------------------
// Diagram reference (mirrors HTML diagram titles):
//   A. Servo + failsafe core
//   B. Input modes (PPM / CRSF / ESP-NOW)
//   C. Transport receivers (ISR, CRSF parser, ESP-NOW callback)
//   D. CLI + tuning helpers
// Use editor folding on the section separators to hide implementation details.
// ============================================================================

// -----------------------------------------------------------------------------
// USER CONFIGURATION — RX (DRONE SIDE)
// -----------------------------------------------------------------------------
const char* ELRS_BIND_PHRASE = "gimbal";  // pour mémoire
// MAC de l'ESP32 TX (WiFi.macAddress() sur le TX -> 3C:8A:1F:A6:5A:80)
const uint8_t ESP_NOW_PEER_MAC[6] = {0x3C, 0x8A, 0x1F, 0xA6, 0x5A, 0x80};

// ---------------------------------------------------------------------------
// SECTION A — Servo core & smoothing knobs
// ---------------------------------------------------------------------------
// Servo setup
Servo panServo;
Servo tiltServo;

// Pin definitions
const int PPM_INPUT_PIN = 2;    // PPM signal (mode PPM local)
const int PAN_PIN       = 18;   // Pan servo output
const int TILT_PIN      = 19;   // Tilt servo output

// PPM processing variables (pour mode PPM local)
volatile unsigned long ppmPulseStart = 0;
volatile unsigned long ppmChannels[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
volatile int   ppmChannelIndex  = 0;
volatile bool  ppmFrameComplete = false;
volatile unsigned long lastValidFrame = 0;

// Servo limits and positions
const int PAN_CENTER = 90;
const int PAN_MIN    = 0;
const int PAN_MAX    = 180;

const int TILT_CENTER = 90;
const int TILT_MIN    = 45;
const int TILT_MAX    = 135;

// Current positions
int currentPan  = PAN_CENTER;
int currentTilt = TILT_CENTER;
int targetPan   = PAN_CENTER;
int targetTilt  = TILT_CENTER;

// Timing / failsafe
const unsigned long PPM_PULSE_MIN  = 900;
const unsigned long PPM_PULSE_MAX  = 2100;
const unsigned long PPM_FRAME_GAP  = 4000;
const unsigned long SIGNAL_TIMEOUT = 500;   // ms – timeout radio ou PPM

// Smoothing and deadband
int   deadband     = 3;
int   smoothFactor = 5;

// Link smoothing (si tu veux lisser un peu les paquets reçus)
const float LINK_SMOOTH_ALPHA = 0.25f;

// -----------------------------------------------------------------------------
// SECTION B — Input mode selection
// -----------------------------------------------------------------------------
enum InputMode {
  MODE_PPM_LOCAL = 0,   // lit PPM sur GPIO2 (test, filaire)
  MODE_CRSF_ELRS = 1,   // lit CRSF sur UART1 (depuis module ELRS RX)
  MODE_ESPNOW    = 2    // lit ESP-NOW (depuis TX ESP32)
};

InputMode inputMode       = MODE_CRSF_ELRS;
const int MODE_SELECT_PIN = -1;
const bool MODE_SELECT_PULLUP = true;

// UART (CRSF / ELRS)
HardwareSerial elrsSerial(1);
const int ELRS_UART_TX_PIN = -1;        // Not used on RX (RX only)
const int ELRS_UART_RX_PIN = 17;        // UART RX in
const unsigned long ELRS_UART_BAUD = 420000;
bool elrsReady = false;

// CRSF constants
const uint8_t  CRSF_DEVICE_ADDRESS        = 0xC8;
const uint8_t  CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
const uint8_t  CRSF_RC_PAYLOAD_LEN        = 22;
const uint8_t  CRSF_RC_CHANNEL_COUNT      = 16;
const uint8_t  CRSF_MAX_FRAME_LEN         = 64;

// ESP-NOW settings
bool espNowReady = false;

// -----------------------------------------------------------------------------
// Packet commun TX/RX (ESP-NOW only; CRSF uses native frames)
// -----------------------------------------------------------------------------
struct __attribute__((packed)) ControlPacket {
  uint16_t header;   // 0xA55A
  uint16_t pan;      // degrees
  uint16_t tilt;     // degrees
  uint16_t flags;    // bit0: valid frame
  uint16_t checksum; // XOR
};

uint16_t packetChecksum(const ControlPacket& pkt) {
  return pkt.header ^ pkt.pan ^ pkt.tilt ^ pkt.flags ^ 0x55AA;
}

// pour smoothing des paquets
float linkPan  = PAN_CENTER;
float linkTilt = TILT_CENTER;

// --- FORWARD DECLARATIONS ---
void IRAM_ATTR ppmInterrupt();
void processPPMFrame();
void updateServos();
void processSerialCommand(String command);
void testServoMovement();

void initEspNowRX();
void initElrsUartRX();
void setInputMode(InputMode newMode, bool announce);
const char* modeName(InputMode m);
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len);
void readCrsfPackets();
void applyCrsfChannels(const uint8_t* payload);
uint8_t crsfCalcCRC(const uint8_t* data, uint8_t len);
int crsfToMicroseconds(uint16_t value);
int microsecondsToDegrees(int microseconds, int degMin, int degMax);

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // 1) Attach servos immediately so failsafe centers are respected.
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  currentPan  = constrain(PAN_CENTER,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(TILT_CENTER, TILT_MIN, TILT_MAX);
  targetPan   = currentPan;
  targetTilt  = currentTilt;
  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  // 2) Arm the optional PPM test input (useful when ESP32 is wired straight to goggles).
  pinMode(PPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, CHANGE);

  Serial.println("Skyzone 04X PRO Head Tracker RX (Drone)");
  Serial.println("=====================================");
  Serial.println("Modes: PPM local, CRSF/ELRS, ESP-NOW");

  // 3) Optional hardware selector (jumper/switch) chooses between wired PPM and CRSF.
  if (MODE_SELECT_PIN >= 0) {
    pinMode(MODE_SELECT_PIN, MODE_SELECT_PULLUP ? INPUT_PULLUP : INPUT);
    bool high = digitalRead(MODE_SELECT_PIN);
    inputMode = high ? MODE_CRSF_ELRS : MODE_PPM_LOCAL;
  }

  setInputMode(inputMode, false);

  Serial.printf("Initial mode: %s\n", modeName(inputMode));
  Serial.println("Commands: 'center', 'debug', 'limits', 'test',");
  Serial.println("          'deadband X', 'smooth Y', 'mode ppm|crsf|espnow'");
  delay(500);
}

// -----------------------------------------------------------------------------
// SECTION C — Main loop + failsafe handling
// -----------------------------------------------------------------------------
void loop() {
  bool haveSignal = false;
  unsigned long now = millis();

  // 1) Poll whichever transport is active and update lastValidFrame.
  if (inputMode == MODE_PPM_LOCAL) {
    if (ppmFrameComplete) {
      processPPMFrame();
      ppmFrameComplete = false;
      lastValidFrame = now;
    }
    haveSignal = (now - lastValidFrame <= SIGNAL_TIMEOUT);

  } else if (inputMode == MODE_CRSF_ELRS) {
    readCrsfPackets();  // met à jour targetPan/tilt + lastValidFrame
    haveSignal = (now - lastValidFrame <= SIGNAL_TIMEOUT);

  } else if (inputMode == MODE_ESPNOW) {
    // les paquets ESP-NOW sont traités dans le callback
    haveSignal = (now - lastValidFrame <= SIGNAL_TIMEOUT);
  }

  // 2) Failsafe : recentrer si plus de signal
  if (!haveSignal) {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
  }

  // 3) Slew servos toward target (keeps motion smooth even if packets jump).
  updateServos();

  // 4) Serial CLI for runtime tuning / diagnostics.
  if (Serial.available()) {
    String cmd = Serial.readString();
    cmd.trim();
    processSerialCommand(cmd);
  }

  delay(20);  // ~50 Hz
}

// -----------------------------------------------------------------------------
// SECTION D — Transport receivers
// -----------------------------------------------------------------------------
// --- PPM ISR (local wired fallback) ---
void IRAM_ATTR ppmInterrupt() {
  static unsigned long lastPulseTime = 0;
  unsigned long currentTime = micros();
  bool pinState = digitalRead(PPM_INPUT_PIN);

  if (pinState == HIGH) {
    lastPulseTime = currentTime;
  } else {
    unsigned long pulseDuration = currentTime - lastPulseTime;
    if (pulseDuration > PPM_FRAME_GAP) {
      ppmChannelIndex  = 0;
      ppmFrameComplete = true;
    } else if (pulseDuration >= PPM_PULSE_MIN && pulseDuration <= PPM_PULSE_MAX) {
      if (ppmChannelIndex < 8) {
        ppmChannels[ppmChannelIndex] = pulseDuration;
        ppmChannelIndex++;
      }
    }
  }
}

void processPPMFrame() {
  unsigned long panPulse  = ppmChannels[4];  // CH5
  unsigned long tiltPulse = ppmChannels[5];  // CH6

  if (panPulse < PPM_PULSE_MIN || panPulse > PPM_PULSE_MAX)   panPulse = 1500;
  if (tiltPulse < PPM_PULSE_MIN || tiltPulse > PPM_PULSE_MAX) tiltPulse = 1500;

  int newPan  = map(panPulse,  1000, 2000, PAN_MIN,  PAN_MAX);
  int newTilt = map(tiltPulse, 1000, 2000, TILT_MIN, TILT_MAX);

  if (abs(newPan  - PAN_CENTER)  < deadband) newPan  = PAN_CENTER;
  if (abs(newTilt - TILT_CENTER) < deadband) newTilt = TILT_CENTER;

  targetPan  = constrain(newPan,  PAN_MIN,  PAN_MAX);
  targetTilt = constrain(newTilt, TILT_MIN, TILT_MAX);

  static int frameCount = 0;
  if (frameCount++ % 50 == 0) {
    Serial.printf("[PPM] Pan: %lu µs -> %d°, Tilt: %lu µs -> %d°\n",
                  panPulse, targetPan, tiltPulse, targetTilt);
  }
}

// -----------------------------------------------------------------------------
// ESP-NOW RX
// -----------------------------------------------------------------------------
void onEspNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  (void)info; // Source MAC lives inside info->src_addr if needed later.
  if (len != sizeof(ControlPacket)) return;

  ControlPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.header != 0xA55A) return;
  if (pkt.checksum != packetChecksum(pkt)) return;

  // lissage léger du lien
  linkPan  += (pkt.pan  - linkPan)  * LINK_SMOOTH_ALPHA;
  linkTilt += (pkt.tilt - linkTilt) * LINK_SMOOTH_ALPHA;

  targetPan  = constrain(static_cast<int>(linkPan  + 0.5f), PAN_MIN,  PAN_MAX);
  targetTilt = constrain(static_cast<int>(linkTilt + 0.5f), TILT_MIN, TILT_MAX);

  lastValidFrame = millis();
}

void initEspNowRX() {
  if (espNowReady) return;
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  // pas obligé d'ajouter un peer pour RX seulement, mais on peut si tu veux du bidirectionnel
  espNowReady = true;
  Serial.println("ESP-NOW RX ready.");
}

// -----------------------------------------------------------------------------
// UART RX (ELRS ou filaire)
// -----------------------------------------------------------------------------
void initElrsUartRX() {
  if (elrsReady) return;
  if (ELRS_UART_RX_PIN < 0) {
    Serial.println("ELRS UART RX pin not defined!");
    return;
  }
  elrsSerial.begin(ELRS_UART_BAUD, SERIAL_8N1, ELRS_UART_RX_PIN, ELRS_UART_TX_PIN);
  elrsReady = true;
  Serial.println("ELRS CRSF UART RX ready.");
}

// Stateful parser for CRSF RC_CHANNELS_PACKED frames arriving over UART1.
void readCrsfPackets() {
  if (!elrsReady) initElrsUartRX();
  if (!elrsReady) return;

  static enum { WAIT_ADDR, WAIT_LEN, READ_FRAME } state = WAIT_ADDR;
  static uint8_t frameLen = 0;
  static uint8_t frameIdx = 0;
  static uint8_t frameBuf[CRSF_MAX_FRAME_LEN];

  while (elrsSerial.available()) {
    uint8_t byteIn = elrsSerial.read();
    switch (state) {
      case WAIT_ADDR:
        if (byteIn == CRSF_DEVICE_ADDRESS) {
          state = WAIT_LEN;
        }
        break;
      case WAIT_LEN:
        if (byteIn < 2 || byteIn > CRSF_MAX_FRAME_LEN) {
          state = WAIT_ADDR;
          break;
        }
        frameLen = byteIn;
        frameIdx = 0;
        state = READ_FRAME;
        break;
      case READ_FRAME:
        if (frameIdx < CRSF_MAX_FRAME_LEN) {
          frameBuf[frameIdx++] = byteIn;
        }
        if (frameIdx >= frameLen) {
          // frameBuf contains [type][payload...][crc]
          uint8_t type = frameBuf[0];
          if (frameLen >= 2) {
            uint8_t crc = frameBuf[frameLen - 1];
            uint8_t calc = crsfCalcCRC(frameBuf, frameLen - 1);
            if (calc == crc) {
              uint8_t payloadLen = frameLen - 2;
              if (type == CRSF_FRAMETYPE_RC_CHANNELS && payloadLen == CRSF_RC_PAYLOAD_LEN) {
                applyCrsfChannels(&frameBuf[1]);
              }
            }
          }
          state = WAIT_ADDR;
        }
        break;
    }
  }
}

// -----------------------------------------------------------------------------
// SECTION E — Servo motion & CLI
// -----------------------------------------------------------------------------
void updateServos() {
  // Slew limiter: move gradually toward targets to avoid gimbal overshoot.
  int panDiff  = targetPan  - currentPan;
  int tiltDiff = targetTilt - currentTilt;

  if (abs(panDiff) > 0) {
    currentPan += panDiff / smoothFactor;
    if (abs(panDiff) < smoothFactor) currentPan = targetPan;
  }
  if (abs(tiltDiff) > 0) {
    currentTilt += tiltDiff / smoothFactor;
    if (abs(tiltDiff) < smoothFactor) currentTilt = targetTilt;
  }

  currentPan  = constrain(currentPan,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);

  panServo.write(currentPan);
  tiltServo.write(currentTilt);
}

// -----------------------------------------------------------------------------
// Serial commands RX
// -----------------------------------------------------------------------------
void processSerialCommand(String command) {
  command.toLowerCase();

  if (command == "center") {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
    Serial.println("Moving to center position");

  } else if (command.startsWith("mode")) {
    if (command.endsWith("pwm") || command.endsWith("ppm")) {
      setInputMode(MODE_PPM_LOCAL, true);
    } else if (command.endsWith("crsf") || command.endsWith("elrs") || command.endsWith("uart")) {
      setInputMode(MODE_CRSF_ELRS, true);
    } else if (command.endsWith("espnow")) {
      setInputMode(MODE_ESPNOW, true);
    } else {
      Serial.println("Unknown mode. Use 'mode ppm', 'mode crsf', or 'mode espnow'.");
    }

  } else if (command == "debug") {
    Serial.println("PPM Channel Values (last frame):");
    for (int i = 0; i < 8; i++) {
      Serial.printf("Channel %d: %lu µs\n", i + 1, ppmChannels[i]);
    }
    Serial.printf("Current Position - Pan: %d, Tilt: %d\n", currentPan,  currentTilt);
    Serial.printf("Target Position  - Pan: %d, Tilt: %d\n", targetPan,   targetTilt);

  } else if (command == "limits") {
    Serial.println("Servo Limits:");
    Serial.printf("Pan: %d° to %d° (Center: %d°)\n", PAN_MIN, PAN_MAX, PAN_CENTER);
    Serial.printf("Tilt: %d° to %d° (Center: %d°)\n", TILT_MIN, TILT_MAX, TILT_CENTER);

  } else if (command.startsWith("deadband")) {
    int space = command.indexOf(' ');
    if (space > 0) {
      int newDeadband = command.substring(space + 1).toInt();
      if (newDeadband >= 0 && newDeadband <= 50) {
        deadband = newDeadband;
        Serial.printf("Deadband set to: %d\n", deadband);
      } else {
        Serial.println("Invalid deadband value (must be 0-50).");
      }
    }

  } else if (command.startsWith("smooth")) {
    int space = command.indexOf(' ');
    if (space > 0) {
      int newSmooth = command.substring(space + 1).toInt();
      if (newSmooth >= 1 && newSmooth <= 50) {
        smoothFactor = newSmooth;
        Serial.printf("Smooth factor set to: %d\n", smoothFactor);
      } else {
        Serial.println("Invalid smooth factor value (must be 1-50).");
      }
    }

  } else if (command == "test") {
    Serial.println("Testing servo movement...");
    testServoMovement();

  } else {
    Serial.println("Unknown command. Available: center, debug, limits, test,");
    Serial.println("deadband X, smooth Y, mode ppm|crsf|espnow");
  }
}

void testServoMovement() {
  Serial.println("Testing pan movement...");
  for (int pos = PAN_CENTER; pos <= PAN_MAX; pos += 5) {
    panServo.write(pos);
    delay(500);
  }
  for (int pos = PAN_MAX; pos >= PAN_MIN; pos -= 5) {
    panServo.write(pos);
    delay(500);
  }
  for (int pos = PAN_MIN; pos <= PAN_CENTER; pos += 5) {
    panServo.write(pos);
    delay(500);
  }

  Serial.println("Testing tilt movement...");
  for (int pos = TILT_CENTER; pos <= TILT_MAX; pos += 3) {
    tiltServo.write(pos);
    delay(500);
  }
  for (int pos = TILT_MAX; pos >= TILT_MIN; pos -= 3) {
    tiltServo.write(pos);
    delay(50);
  }
  for (int pos = TILT_MIN; pos <= TILT_CENTER; pos += 3) {
    tiltServo.write(pos);
    delay(500);
  }

  Serial.println("Test complete - returning to center");
  currentPan  = PAN_CENTER;
  currentTilt = TILT_CENTER;
  targetPan   = PAN_CENTER;
  targetTilt  = TILT_CENTER;
  panServo.write(currentPan);
  tiltServo.write(currentTilt);
}

// -----------------------------------------------------------------------------
// Mode helpers
// -----------------------------------------------------------------------------
void setInputMode(InputMode newMode, bool announce) {
  inputMode = newMode;

  switch (inputMode) {
    case MODE_ESPNOW:
      initEspNowRX();
      break;
    case MODE_CRSF_ELRS:
      initElrsUartRX();
      break;
    case MODE_PPM_LOCAL:
    default:
      break;
  }

  if (announce) {
    Serial.printf("Input mode set to %s\n", modeName(inputMode));
  }
}

const char* modeName(InputMode m) {
  switch (m) {
    case MODE_PPM_LOCAL: return "PPM local";
    case MODE_CRSF_ELRS: return "CRSF / ELRS";
    case MODE_ESPNOW:    return "ESP-NOW";
    default:             return "Unknown";
  }
}

void applyCrsfChannels(const uint8_t* payload) {
  uint32_t bitBuffer = 0;
  uint8_t bitsInBuffer = 0;
  uint16_t channels[CRSF_RC_CHANNEL_COUNT] = {0};
  uint8_t byteIndex = 0;

  for (int ch = 0; ch < CRSF_RC_CHANNEL_COUNT; ++ch) {
    while (bitsInBuffer < 11 && byteIndex < CRSF_RC_PAYLOAD_LEN) {
      bitBuffer |= ((uint32_t)payload[byteIndex++]) << bitsInBuffer;
      bitsInBuffer += 8;
    }
    channels[ch] = bitBuffer & 0x7FF;
    bitBuffer >>= 11;
    bitsInBuffer -= 11;
  }

  int panMicro  = crsfToMicroseconds(channels[0]);
  int tiltMicro = crsfToMicroseconds(channels[1]);
  int panDeg    = microsecondsToDegrees(panMicro, PAN_MIN, PAN_MAX);
  int tiltDeg   = microsecondsToDegrees(tiltMicro, TILT_MIN, TILT_MAX);

  linkPan  += (panDeg  - linkPan)  * LINK_SMOOTH_ALPHA;
  linkTilt += (tiltDeg - linkTilt) * LINK_SMOOTH_ALPHA;

  targetPan  = constrain(static_cast<int>(linkPan  + 0.5f), PAN_MIN,  PAN_MAX);
  targetTilt = constrain(static_cast<int>(linkTilt + 0.5f), TILT_MIN, TILT_MAX);

  lastValidFrame = millis();
}

uint8_t crsfCalcCRC(const uint8_t* data, uint8_t len) {
  const uint8_t POLY = 0xD5;
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x80) crc = (crc << 1) ^ POLY;
      else crc <<= 1;
    }
  }
  return crc;
}

int crsfToMicroseconds(uint16_t value) {
  value = constrain(value, 172, 1811);
  long micro = map(value, 172, 1811, 988, 2012);
  return (int)constrain(micro, 1000L, 2000L);
}

int microsecondsToDegrees(int microseconds, int degMin, int degMax) {
  microseconds = constrain(microseconds, 1000, 2000);
  long deg = map(microseconds, 1000, 2000, degMin, degMax);
  return (int)constrain(deg, degMin, degMax);
}
