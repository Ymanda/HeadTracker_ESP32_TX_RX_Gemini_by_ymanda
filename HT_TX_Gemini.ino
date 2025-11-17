#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>

// -----------------------------------------------------------------------------
// USER CONFIGURATION — EDIT BEFORE FLASHING (TX = GOGGLES SIDE)
// -----------------------------------------------------------------------------
const char* ELRS_BIND_PHRASE = "CHANGE_ME";  // Pour rappel, même wording que dans ExpressLRS
// MAC de l'ESP32 RX (print WiFi.macAddress() sur le RX et colle-le ici)
const uint8_t ESP_NOW_PEER_MAC[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

// Servo setup (pour tests locaux sur la table, sinon tu peux les ignorer)
Servo panServo;
Servo tiltServo;

// Pin definitions
const int PPM_INPUT_PIN = 2;   // PPM signal from Skyzone HT OUT jack
const int PAN_PIN       = 18;  // Pan servo output (TEST LOCAL)
const int TILT_PIN      = 19;  // Tilt servo output (TEST LOCAL)

// PPM processing variables
volatile unsigned long ppmPulseStart = 0;
volatile unsigned long ppmChannels[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
volatile int   ppmChannelIndex   = 0;
volatile bool  ppmFrameComplete  = false;
volatile unsigned long lastValidFrame = 0;

// Servo limits and positions
// Pan servo: 270° range mapped to 0-180° servo output
const int PAN_CENTER = 90;     // Center position
const int PAN_MIN    = 0;      // Full left
const int PAN_MAX    = 180;    // Full right

// Tilt servo: 60° range mapped to servo output
const int TILT_CENTER = 90;    // Center position
const int TILT_MIN    = 45;    // Looking down (30° below center)
const int TILT_MAX    = 135;   // Looking up (30° above center)

// Current positions
int currentPan  = PAN_CENTER;
int currentTilt = TILT_CENTER;
int targetPan   = PAN_CENTER;
int targetTilt  = TILT_CENTER;

// PPM processing constants
const unsigned long PPM_PULSE_MIN  = 900;   // Min valid pulse width (µs)
const unsigned long PPM_PULSE_MAX  = 2100;  // Max valid pulse width (µs)
const unsigned long PPM_FRAME_GAP  = 4000;  // Gap between frames (µs)
const unsigned long PPM_TIMEOUT    = 1000;  // Timeout for PPM signal (ms)

// Smoothing and deadband
int   deadband     = 3;        // Deadband around center position.
int   smoothFactor = 5;        // Lower = more responsive, higher = smoother.
const float WIRELESS_SMOOTH_ALPHA = 0.35f;  // EWMA used when sending remotely

// -----------------------------------------------------------------------------
// Multi-transport configuration
// -----------------------------------------------------------------------------
enum OutputMode {
  MODE_WIRED_PWM = 0,   // PPM -> PWM local (test bench)
  MODE_UART_ELRS = 1,   // PPM -> UART (vers module ELRS ou autre ESP32 filaire)
  MODE_ESPNOW    = 2    // PPM -> ESP-NOW (vers ESP32 RX sur drone)
};

OutputMode outputMode       = MODE_WIRED_PWM;
const int  MODE_SELECT_PIN  = -1;     // si tu veux un switch physique
const bool MODE_SELECT_PULLUP = true;

// UART (ELRS bridge) settings
HardwareSerial elrsSerial(1);
const int ELRS_UART_TX_PIN   = 17;
const int ELRS_UART_RX_PIN   = -1;     // Not used on TX
const unsigned long ELRS_UART_BAUD = 115200;
bool elrsReady = false;

// ESP-NOW settings
bool    espNowReady   = false;
uint8_t espNowPeer[6] = {0};          // Populated from ESP_NOW_PEER_MAC
const unsigned long WIRELESS_SEND_INTERVAL_MS = 10;
unsigned long lastWirelessSend = 0;

struct __attribute__((packed)) ControlPacket {
  uint16_t header;   // Fixed marker 0xA55A
  uint16_t pan;      // Degrees
  uint16_t tilt;     // Degrees
  uint16_t flags;    // bit0: valid frame
  uint16_t checksum; // simple XOR checksum
};

float wirelessPan  = PAN_CENTER;
float wirelessTilt = TILT_CENTER;

// --- FORWARD DECLARATIONS ---
void processSerialCommand(String command);
void testServoMovement();
void IRAM_ATTR ppmInterrupt();
void setOutputMode(OutputMode newMode, bool announce);
void handleOutputs(bool ppmValid);
void transmitWirelessTargets(bool ppmValid);
void initEspNow();
void initElrsUart();
const char* modeName(OutputMode mode);
uint16_t packetChecksum(const ControlPacket& pkt);
void processPPMFrame();
void updateServos();

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  memcpy(espNowPeer, ESP_NOW_PEER_MAC, sizeof(espNowPeer));

  // Initialize servos (local test)
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  // Set to center position
  currentPan  = constrain(currentPan,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);
  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  Serial.println("Skyzone 04X PRO Head Tracker TX (Goggles)");
  Serial.println("========================================");
  Serial.println("Connect HT OUT jack to GPIO 2");
  Serial.println("Pan servo: 270° range");
  Serial.println("Tilt servo: 60° range");
  Serial.println("Starting in center position...");

  // Setup PPM input interrupt
  pinMode(PPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, CHANGE);

  // Optional hardware switch between wired/wireless outputs
  if (MODE_SELECT_PIN >= 0) {
    pinMode(MODE_SELECT_PIN, MODE_SELECT_PULLUP ? INPUT_PULLUP : INPUT);
    bool high = digitalRead(MODE_SELECT_PIN);
    outputMode = high ? MODE_UART_ELRS : MODE_WIRED_PWM;
  }

  setOutputMode(outputMode, false);

  Serial.println("Ready for PPM input!");
  Serial.printf("ELRS bind phrase reminder: %s\n", ELRS_BIND_PHRASE);
  Serial.printf("ESP-NOW peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                espNowPeer[0], espNowPeer[1], espNowPeer[2],
                espNowPeer[3], espNowPeer[4], espNowPeer[5]);
  Serial.println("Commands: 'center', 'debug', 'limits', 'test', ");
  Serial.println("          'deadband X', 'smooth Y', 'mode pwm|uart|espnow'");

  delay(1000);
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop() {
  // Process PPM data
  if (ppmFrameComplete) {
    processPPMFrame();
    ppmFrameComplete = false;
    lastValidFrame = millis();
  }

  // Check for PPM timeout
  if (millis() - lastValidFrame > PPM_TIMEOUT) {
    // No PPM signal - return to center
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
  }

  bool ppmValid = (millis() - lastValidFrame) <= PPM_TIMEOUT;

  // Handle outputs depending on selected mode
  handleOutputs(ppmValid);

  // Process serial commands
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    processSerialCommand(command);
  }

  delay(20); // 50Hz update rate
}

// -----------------------------------------------------------------------------
// PPM interrupt handler (TX side = same que ta version)
// -----------------------------------------------------------------------------
void IRAM_ATTR ppmInterrupt() {
  static unsigned long lastPulseTime = 0;
  unsigned long currentTime = micros();
  bool pinState = digitalRead(PPM_INPUT_PIN);

  if (pinState == HIGH) {
    // Rising edge - start measuring pulse
    lastPulseTime = currentTime;
  } else {
    // Falling edge - end of pulse
    unsigned long pulseDuration = currentTime - lastPulseTime;

    if (pulseDuration > PPM_FRAME_GAP) {
      // Frame sync detected - start new frame
      ppmChannelIndex = 0;
      ppmFrameComplete = true;
    } else if (pulseDuration >= PPM_PULSE_MIN && pulseDuration <= PPM_PULSE_MAX) {
      // Valid channel pulse
      if (ppmChannelIndex < 8) {
        ppmChannels[ppmChannelIndex] = pulseDuration;
        ppmChannelIndex++;
      }
    }
  }
}

void processPPMFrame() {
  // Skyzone 04X PRO outputs head tracking on channels 5 and 6
  // Channel 5 (index 4) = Pan (yaw)
  // Channel 6 (index 5) = Tilt (pitch)

  unsigned long panPulse  = ppmChannels[4];  // Channel 5
  unsigned long tiltPulse = ppmChannels[5];  // Channel 6

  // Validate pulse widths
  if (panPulse < PPM_PULSE_MIN || panPulse > PPM_PULSE_MAX)   panPulse = 1500;
  if (tiltPulse < PPM_PULSE_MIN || tiltPulse > PPM_PULSE_MAX) tiltPulse = 1500;

  // Convert PPM to servo positions  (1000..2000µs -> limits)
  int newPan  = map(panPulse,  1000, 2000, PAN_MIN,  PAN_MAX);
  int newTilt = map(tiltPulse, 1000, 2000, TILT_MIN, TILT_MAX);

  // Apply deadband around center
  if (abs(newPan  - PAN_CENTER)  < deadband) newPan  = PAN_CENTER;
  if (abs(newTilt - TILT_CENTER) < deadband) newTilt = TILT_CENTER;

  // Constrain to limits
  targetPan  = constrain(newPan,  PAN_MIN,  PAN_MAX);
  targetTilt = constrain(newTilt, TILT_MIN, TILT_MAX);

  // Debug output (every 50 frames to avoid spam)
  static int frameCount = 0;
  if (frameCount++ % 50 == 0) {
    Serial.printf("Pan: %lu µs -> %d°, Tilt: %lu µs -> %d°\n",
                  panPulse, targetPan, tiltPulse, targetTilt);
  }
}

void updateServos() {
  // Smooth movement towards target
  int panDiff  = targetPan  - currentPan;
  int tiltDiff = targetTilt - currentTilt;

  // Apply smoothing
  if (abs(panDiff) > 0) {
    currentPan += panDiff / smoothFactor;
    if (abs(panDiff) < smoothFactor) currentPan = targetPan;
  }

  if (abs(tiltDiff) > 0) {
    currentTilt += tiltDiff / smoothFactor;
    if (abs(tiltDiff) < smoothFactor) currentTilt = targetTilt;
  }

  // Constrain to absolute limits (safety)
  currentPan  = constrain(currentPan,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);

  // Update servos (ONLY for local testing on TX)
  panServo.write(currentPan);
  tiltServo.write(currentTilt);
}

// -----------------------------------------------------------------------------
// Serial commands (identiques à ta version)
// -----------------------------------------------------------------------------
void processSerialCommand(String command) {
  command.toLowerCase();

  if (command == "center") {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
    Serial.println("Moving to center position");

  } else if (command.startsWith("mode")) {
    if (command.endsWith("pwm")) {
      setOutputMode(MODE_WIRED_PWM, true);
    } else if (command.endsWith("uart")) {
      setOutputMode(MODE_UART_ELRS, true);
    } else if (command.endsWith("espnow")) {
      setOutputMode(MODE_ESPNOW, true);
    } else {
      Serial.println("Unknown mode. Use 'mode pwm', 'mode uart', or 'mode espnow'.");
    }

  } else if (command == "debug") {
    Serial.println("PPM Channel Values:");
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
    Serial.println("Watch your servos and note the angle values just before they stall.");
    testServoMovement();

  } else {
    Serial.println("Unknown command. Available: center, debug, limits, test,");
    Serial.println("deadband X, smooth Y, mode pwm|uart|espnow");
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

// ---------------------------------------------------------------------------
// Multi-transport helpers (TX)
// ---------------------------------------------------------------------------
void handleOutputs(bool ppmValid) {
  if (outputMode == MODE_WIRED_PWM) {
    // Just drive local servos
    updateServos();
    return;
  }

  if (millis() - lastWirelessSend >= WIRELESS_SEND_INTERVAL_MS) {
    transmitWirelessTargets(ppmValid);
    lastWirelessSend = millis();
  }
}

void transmitWirelessTargets(bool ppmValid) {
  // smoothing pour le lien radio (pas les servos locaux)
  wirelessPan  += (targetPan  - wirelessPan)  * WIRELESS_SMOOTH_ALPHA;
  wirelessTilt += (targetTilt - wirelessTilt) * WIRELESS_SMOOTH_ALPHA;

  ControlPacket pkt;
  pkt.header = 0xA55A;
  pkt.pan    = constrain(static_cast<int>(wirelessPan  + 0.5f), PAN_MIN,  PAN_MAX);
  pkt.tilt   = constrain(static_cast<int>(wirelessTilt + 0.5f), TILT_MIN, TILT_MAX);
  pkt.flags  = ppmValid ? 0x0001 : 0x0000;
  pkt.checksum = packetChecksum(pkt);

  if (outputMode == MODE_UART_ELRS) {
    if (!elrsReady) initElrsUart();
    if (elrsReady) {
      elrsSerial.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
    }
  } else if (outputMode == MODE_ESPNOW) {
    if (!espNowReady) initEspNow();
    if (espNowReady) {
      esp_now_send(espNowPeer, reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
    }
  }
}

void setOutputMode(OutputMode newMode, bool announce) {
  outputMode = newMode;

  switch (outputMode) {
    case MODE_UART_ELRS:
      initElrsUart();
      break;
    case MODE_ESPNOW:
      initEspNow();
      break;
    default:
      break;
  }

  if (announce) {
    Serial.printf("Output mode set to %s\n", modeName(outputMode));
  }
}

void initElrsUart() {
  if (elrsReady) return;
  if (ELRS_UART_TX_PIN < 0) {
    Serial.println("ELRS UART TX pin not defined!");
    return;
  }
  elrsSerial.begin(ELRS_UART_BAUD, SERIAL_8N1, ELRS_UART_RX_PIN, ELRS_UART_TX_PIN);
  elrsReady = true;
  Serial.println("ELRS UART bridge (TX) ready.");
}

void initEspNow() {
  if (espNowReady) return;
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, espNowPeer, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer!");
    return;
  }
  espNowReady = true;
  Serial.println("ESP-NOW link (TX) ready.");
}

const char* modeName(OutputMode mode) {
  switch (mode) {
    case MODE_WIRED_PWM: return "Wired PWM";
    case MODE_UART_ELRS: return "UART ELRS";
    case MODE_ESPNOW:    return "ESP-NOW";
    default:             return "Unknown";
  }
}

uint16_t packetChecksum(const ControlPacket& pkt) {
  return pkt.header ^ pkt.pan ^ pkt.tilt ^ pkt.flags ^ 0x55AA;
}
