
#include <PPMReader.h>
#include <PPMEncoder.h>

// Input pin for PPM from goggles
const byte PPM_INPUT_PIN = 2;   // Must be interrupt capable
const byte PPM_OUTPUT_PIN = 3;  // Any digital output

PPMReader ppmReader(PPM_INPUT_PIN, 8);  // Read up to 8 channels
PPMEncoder ppmEncoder(PPM_OUTPUT_PIN, 8); // Send 8 channels

void setup() {
  for (byte ch = 0; ch < 8; ch++) {
    ppmEncoder.setChannel(ch, 1500); // Neutral default
  }
}

void loop() {
  // Read CH5 and CH6 from goggles (index 4 and 5)
  int ch5 = ppmReader.latestChannelValue(4);
  int ch6 = ppmReader.latestChannelValue(5);

  // Remap to CH1 and CH2 if valid
  if (ch5 > 800 && ch5 < 2200) ppmEncoder.setChannel(0, ch5); // CH1
  if (ch6 > 800 && ch6 < 2200) ppmEncoder.setChannel(1, ch6); // CH2

  delay(5);  // Stability delay
}
