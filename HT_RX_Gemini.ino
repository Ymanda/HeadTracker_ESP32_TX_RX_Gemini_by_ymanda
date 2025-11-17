#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>

// -----------------------------------------------------------------------------
// USER CONFIGURATION — RX (DRONE SIDE)
// -----------------------------------------------------------------------------
const char* ELRS_BIND_PHRASE = "CHANGE_ME";  // pour mémoire
// MAC de l'ESP32 TX (si tu veux unidirectionnel ESP-NOW; ici pas obligatoire)
const uint8_t ESP_NOW_PEER_MAC[6] = {0x24,0x6F,0x28,0xAA,0xBB,0xCC};

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
// Modes d'entrée sur RX
// -----------------------------------------------------------------------------
enum InputMode {
  MODE_PPM_LOCAL = 0,   // lit PPM sur GPIO2 (test, filaire)
  MODE_UART_ELRS = 1,   // lit UART1 (depuis module ELRS RX ou ESP32 en filaire)
  MODE_ESPNOW    = 2    // lit ESP-NOW (depuis TX ESP32)
};

InputMode inputMode      = MODE_ESPNOW;
const int MODE_SELECT_PIN = -1;
const bool MODE_SELECT_PULLUP = true;

// UART (ELRS / liaison filaire)
HardwareSerial elrsSerial(1);
const int ELRS_UART_TX_PIN = -1;        // Not used on RX (RX only)
const int ELRS_UART_RX_PIN = 17;        // UART RX in
const unsigned long ELRS_UART_BAUD = 115200;
bool elrsReady = false;

// ESP-NOW settings
bool espNowReady = false;

// -----------------------------------------------------------------------------
// Packet commun TX/RX
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
void readElrsPackets();

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Servos
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  currentPan  = constrain(PAN_CENTER,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(TILT_CENTER, TILT_MIN, TILT_MAX);
  targetPan   = currentPan;
  targetTilt  = currentTilt;
  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  // PPM input
  pinMode(PPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, CHANGE);

  Serial.println("Skyzone 04X PRO Head Tracker RX (Drone)");
  Serial.println("=====================================");
  Serial.println("Modes: PPM local, UART ELRS, ESP-NOW");

  // Mode hardware optionnel
  if (MODE_SELECT_PIN >= 0) {
    pinMode(MODE_SELECT_PIN, MODE_SELECT_PULLUP ? INPUT_PULLUP : INPUT);
    bool high = digitalRead(MODE_SELECT_PIN);
    inputMode = high ? MODE_UART_ELRS : MODE_PPM_LOCAL;
  }

  setInputMode(inputMode, false);

  Serial.printf("Initial mode: %s\n", modeName(inputMode));
  Serial.println("Commands: 'center', 'debug', 'limits', 'test',");
  Serial.println("          'deadband X', 'smooth Y', 'mode ppm|uart|espnow'");
  delay(500);
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop() {
  bool haveSignal = false;
  unsigned long now = millis();

  // 1) Lecture selon le mode
  if (inputMode == MODE_PPM_LOCAL) {
    if (ppmFrameComplete) {
      processPPMFrame();
      ppmFrameComplete = false;
      lastValidFrame = now;
    }
    haveSignal = (now - lastValidFrame <= SIGNAL_TIMEOUT);

  } else if (inputMode == MODE_UART_ELRS) {
    readElrsPackets();  // met à jour targetPan/tilt + lastValidFrame
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

  // 3) Mise à jour des servos
  updateServos();

  // 4) Commandes série
  if (Serial.available()) {
    String cmd = Serial.readString();
    cmd.trim();
    processSerialCommand(cmd);
  }

  delay(20);  // ~50 Hz
}

// -----------------------------------------------------------------------------
// PPM RX (local) pour test / mode filaire
// -----------------------------------------------------------------------------
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
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
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
  Serial.println("ELRS UART RX ready.");
}

void readElrsPackets() {
  if (!elrsReady) initElrsUartRX();
  if (!elrsReady) return;

  while (elrsSerial.available() >= (int)sizeof(ControlPacket)) {
    ControlPacket pkt;
    elrsSerial.readBytes((char*)&pkt, sizeof(pkt));

    if (pkt.header != 0xA55A) continue;
    if (pkt.checksum != packetChecksum(pkt)) continue;

    linkPan  += (pkt.pan  - linkPan)  * LINK_SMOOTH_ALPHA;
    linkTilt += (pkt.tilt - linkTilt) * LINK_SMOOTH_ALPHA;

    targetPan  = constrain(static_cast<int>(linkPan  + 0.5f), PAN_MIN,  PAN_MAX);
    targetTilt = constrain(static_cast<int>(linkTilt + 0.5f), TILT_MIN, TILT_MAX);

    lastValidFrame = millis();
  }
}

// -----------------------------------------------------------------------------
// Servo update
// -----------------------------------------------------------------------------
void updateServos() {
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
    } else if (command.endsWith("uart")) {
      setInputMode(MODE_UART_ELRS, true);
    } else if (command.endsWith("espnow")) {
      setInputMode(MODE_ESPNOW, true);
    } else {
      Serial.println("Unknown mode. Use 'mode ppm', 'mode uart', or 'mode espnow'.");
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
    Serial.println("deadband X, smooth Y, mode ppm|uart|espnow");
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
    case MODE_UART_ELRS:
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
    case MODE_UART_ELRS: return "UART ELRS";
    case MODE_ESPNOW:    return "ESP-NOW";
    default:             return "Unknown";
  }
}
