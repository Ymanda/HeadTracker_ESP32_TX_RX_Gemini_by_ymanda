#include <ESP32_ppm.h>
#include <ESP32Servo.h>

// --- PPM ---
#define PPM_PIN 2  // Your actual PPM input pin
ppmReader myPPM_RX;
int* ppmArray;

// --- Servo ---
#define PAN_PIN 18
#define TILT_PIN 19
Servo panServo;
Servo tiltServo;

void setup() {
  Serial.begin(115200);

  // Start reading PPM
  ppmArray = myPPM_RX.begin(PPM_PIN);
  if (ppmArray == NULL) {
    Serial.println("PPM init failed!");
    while (1);
  }
  myPPM_RX.start();
  Serial.println("PPM Reader started.");

  // Attach servos
  panServo.attach(PAN_PIN, 1000, 2000);   // min/max in microseconds
  tiltServo.attach(TILT_PIN, 1000, 2000); // min/max in microseconds
}

void loop() {
  if (myPPM_RX.newFrame()) {
    int numChannels = ppmArray[0];

    // Print all channels for debug
    Serial.print("Channels: ");
    Serial.print(numChannels);
    Serial.print(" | ");
    for (int i = 1; i <= numChannels; i++) {
      Serial.print("CH");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(ppmArray[i]);
      Serial.print("  ");
    }
    Serial.println();

    // Example: CH5 = PAN, CH6 = TILT
    if (numChannels >= 6) {
      int pan_us  = ppmArray[5];  // channel 6 (index starts at 1)
      int tilt_us = ppmArray[6];  // channel 7

      // Clamp values if needed
      pan_us  = constrain(pan_us, 1000, 2000);
      tilt_us = constrain(tilt_us, 1000, 2000);

      // Move servos
      panServo.writeMicroseconds(pan_us);
      tiltServo.writeMicroseconds(tilt_us);
    }
  }

  delay(10);  // For smoother serial
}
