#include <ESP32Servo.h>

// Servo setup
Servo panServo;
Servo tiltServo;

// Pin definitions
const int PPM_INPUT_PIN = 2;   // PPM signal from Skyzone HT OUT jack
const int PAN_PIN = 18;        // Pan servo output
const int TILT_PIN = 19;       // Tilt servo output

// PPM processing variables
volatile unsigned long ppmPulseStart = 0;
volatile unsigned long ppmChannels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
volatile int ppmChannelIndex = 0;
volatile bool ppmFrameComplete = false;
volatile unsigned long lastValidFrame = 0;

// Servo limits and positions
// Pan servo: 270° range mapped to 0-180° servo output
const int PAN_CENTER = 90;     // Center position
const int PAN_MIN = 0;         // Full left
const int PAN_MAX = 180;       // Full right

// Tilt servo: 60° range mapped to servo output
const int TILT_CENTER = 90;    // Center position
const int TILT_MIN = 45;       // Looking down (30° below center)
const int TILT_MAX = 135;      // Looking up (30° above center)

// Current positions
int currentPan = PAN_CENTER;
int currentTilt = TILT_CENTER;
int targetPan = PAN_CENTER;
int targetTilt = TILT_CENTER;

// PPM processing constants
const unsigned long PPM_PULSE_MIN = 900;   // Minimum valid pulse width (microseconds)
const unsigned long PPM_PULSE_MAX = 2100;  // Maximum valid pulse width (microseconds)
const unsigned long PPM_FRAME_GAP = 4000;  // Gap between frames (microseconds)
const unsigned long PPM_TIMEOUT = 1000;    // Timeout for PPM signal (milliseconds)

// Smoothing and deadband
int deadband = 3;        // Deadband around center position. Renamed from DEADBAND to be non-const.
int smoothFactor = 5;   // Lower = more responsive, higher = smoother. Renamed from SMOOTH_FACTOR to be non-const.

// --- FORWARD DECLARATIONS ---
void processSerialCommand(String command);
void testServoMovement();
void ppmInterrupt();

void setup() {
  Serial.begin(115200);

  // Initialize servos
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  // Set to center position
  currentPan = constrain(currentPan, PAN_MIN, PAN_MAX);
  currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);
  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  Serial.println("Skyzone 04X PRO Head Tracker Interface");
  Serial.println("=====================================");
  Serial.println("Connect HT OUT jack to GPIO 2");
  Serial.println("Pan servo: 270° range");
  Serial.println("Tilt servo: 60° range");
  Serial.println("Starting in center position...");

  // Setup PPM input interrupt
  pinMode(PPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, CHANGE);

  Serial.println("Ready for PPM input!");
  Serial.println("Commands: 'center', 'debug', 'limits', 'test', 'deadband X', 'smooth Y'");

  delay(1000);
}

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
    targetPan = PAN_CENTER;
    targetTilt = TILT_CENTER;
  }

  // Smooth servo movement
  updateServos();

  // Process serial commands
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    processSerialCommand(command);
  }

  delay(20); // 50Hz update rate
}

// PPM interrupt handler
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

  unsigned long panPulse = ppmChannels[4];  // Channel 5
  unsigned long tiltPulse = ppmChannels[5]; // Channel 6

  // Validate pulse widths
  if (panPulse < PPM_PULSE_MIN || panPulse > PPM_PULSE_MAX) panPulse = 1500;
  if (tiltPulse < PPM_PULSE_MIN || tiltPulse > PPM_PULSE_MAX) tiltPulse = 1500;

  // Convert PPM to servo positions
  // PPM: 1000µs = min, 1500µs = center, 2000µs = max
  int newPan = map(panPulse, 1000, 2000, PAN_MIN, PAN_MAX);
  int newTilt = map(tiltPulse, 1000, 2000, TILT_MIN, TILT_MAX);

  // Apply deadband around center
  if (abs(newPan - PAN_CENTER) < deadband) newPan = PAN_CENTER;
  if (abs(newTilt - TILT_CENTER) < deadband) newTilt = TILT_CENTER;

  // Constrain to limits
  targetPan = constrain(newPan, PAN_MIN, PAN_MAX);
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
  int panDiff = targetPan - currentPan;
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
  currentPan = constrain(currentPan, PAN_MIN, PAN_MAX);
  currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);

  // Update servos
  panServo.write(currentPan);
  tiltServo.write(currentTilt);
}

void processSerialCommand(String command) {
  command.toLowerCase();

  if (command == "center") {
    targetPan = PAN_CENTER;
    targetTilt = TILT_CENTER;
    Serial.println("Moving to center position");

  } else if (command == "debug") {
    Serial.println("PPM Channel Values:");
    for (int i = 0; i < 8; i++) {
      Serial.printf("Channel %d: %lu µs\n", i + 1, ppmChannels[i]);
    }
    Serial.printf("Current Position - Pan: %d, Tilt: %d\n", currentPan, currentTilt);
    Serial.printf("Target Position - Pan: %d, Tilt: %d\n", targetPan, targetTilt);

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
    Serial.println("Unknown command. Available: center, debug, limits, test, deadband X, smooth Y");
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
  currentPan = PAN_CENTER;
  currentTilt = TILT_CENTER;
  targetPan = PAN_CENTER;
  targetTilt = TILT_CENTER;
  panServo.write(currentPan);
  tiltServo.write(currentTilt);
}