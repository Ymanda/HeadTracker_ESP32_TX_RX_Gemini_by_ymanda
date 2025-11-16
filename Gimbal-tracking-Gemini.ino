#include <ESP32Servo.h>

// --- PIN DEFINITIONS ---
const int PPM_INPUT_PIN = 2;
const int PAN_PIN = 18;
const int TILT_PIN = 19;

// --- SERVO LIMITS AND POSITIONS ---
// Pan Servo: 180° total range, centered at 90°
const int PAN_CENTER = 90;
const int PAN_MIN = 0;
const int PAN_MAX = 180;

// Tilt Servo: 60° total range. 
// MODIFIED AS REQUESTED: Centered at 30° instead of 90°
const int TILT_CENTER = 30; // Center of the 0-60 range
const int TILT_MIN = 0;
const int TILT_MAX = 60;

// --- TUNING PARAMETERS ---
int deadband = 5;       // Increased to 5 as requested. Stops tiny jitters from input noise.
int smoothFactor = 15;  // !! INCREASED SIGNIFICANTLY !! This is the most likely fix for oscillations.

// --- GLOBAL VARIABLES (Do not change) ---
Servo panServo;
Servo tiltServo;
volatile unsigned long ppmChannels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
volatile int ppmChannelIndex = 0;
volatile bool ppmFrameComplete = false;
volatile unsigned long lastValidFrame = 0;
int currentPan = PAN_CENTER;
int currentTilt = TILT_CENTER; // Will now start at 30
int targetPan = PAN_CENTER;
int targetTilt = TILT_CENTER; // Will now start at 30

// PPM constants
const unsigned long PPM_PULSE_MIN = 900;
const unsigned long PPM_PULSE_MAX = 2100;
const unsigned long PPM_FRAME_GAP = 4000;
const unsigned long PPM_TIMEOUT = 1000;

// Forward Declarations
void ppmInterrupt();
void processPPMFrame();
void updateServos();
void processSerialCommand(String command);
void testServoMovement();

void setup() {
  Serial.begin(115200);

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  // Set to initial center position
  // The constrain calls are a safety check.
  currentPan = constrain(PAN_CENTER, PAN_MIN, PAN_MAX);
  currentTilt = constrain(TILT_CENTER, TILT_MIN, TILT_MAX);
  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  pinMode(PPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, CHANGE);

  Serial.println("Head Tracker Initialized - Test Version");
  Serial.println("---------------------------------------");
  Serial.printf("Tilt Range set to: %d-%d (Center: %d)\n", TILT_MIN, TILT_MAX, TILT_CENTER);
  Serial.printf("Deadband set to: %d\n", deadband);
  Serial.printf("Smooth Factor set to: %d\n", smoothFactor);
  Serial.println("Ready for PPM input.");
}

void loop() {
  if (ppmFrameComplete) {
    processPPMFrame();
    ppmFrameComplete = false;
    lastValidFrame = millis();
  }

  if (millis() - lastValidFrame > PPM_TIMEOUT) {
    targetPan = PAN_CENTER;
    targetTilt = TILT_CENTER;
  }

  updateServos();

  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    processSerialCommand(command);
  }

  delay(20);
}

void IRAM_ATTR ppmInterrupt() {
    // This function remains the same
    static unsigned long lastPulseTime = 0;
    unsigned long currentTime = micros();
    if (digitalRead(PPM_INPUT_PIN) == HIGH) {
        lastPulseTime = currentTime;
    } else {
        unsigned long pulseDuration = currentTime - lastPulseTime;
        if (pulseDuration > PPM_FRAME_GAP) {
            ppmChannelIndex = 0;
            ppmFrameComplete = true;
        } else if (pulseDuration >= PPM_PULSE_MIN && pulseDuration <= PPM_PULSE_MAX && ppmChannelIndex < 8) {
            ppmChannels[ppmChannelIndex++] = pulseDuration;
        }
    }
}

void processPPMFrame() {
  unsigned long panPulse = ppmChannels[4];
  unsigned long tiltPulse = ppmChannels[5];

  // Map PPM pulse to the new servo ranges
  int newPan = map(panPulse, 1000, 2000, PAN_MIN, PAN_MAX);
  int newTilt = map(tiltPulse, 1000, 2000, TILT_MIN, TILT_MAX); // This will now map to 0-60

  // Apply deadband
  if (abs(newPan - PAN_CENTER) < deadband) newPan = PAN_CENTER;
  if (abs(newTilt - TILT_CENTER) < deadband) newTilt = TILT_CENTER;
  
  targetPan = constrain(newPan, PAN_MIN, PAN_MAX);
  targetTilt = constrain(newTilt, TILT_MIN, TILT_MAX);
}

void updateServos() {
    // This function remains the same, but the higher smoothFactor will have a large effect
    int panDiff = targetPan - currentPan;
    if (abs(panDiff) > 0) {
        currentPan += panDiff / smoothFactor;
        if (abs(panDiff) < smoothFactor) currentPan = targetPan;
    }

    int tiltDiff = targetTilt - currentTilt;
    if (abs(tiltDiff) > 0) {
        currentTilt += tiltDiff / smoothFactor;
        if (abs(tiltDiff) < smoothFactor) currentTilt = targetTilt;
    }

    panServo.write(currentPan);
    tiltServo.write(currentTilt);
}

// The processSerialCommand and testServoMovement functions would remain the same
// but are omitted here for brevity. They are not part of the core logic for this problem.
void processSerialCommand(String command){}
void testServoMovement(){}