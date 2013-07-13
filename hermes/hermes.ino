// LED imports
#include "LPD8806.h"
#include "SPI.h"

// Accel imports
#include "Adafruit_LSM303.h"
#include "Wire.h"

int LED_COUNT = 16;
int DATA_PIN = 9;
int CLOCK_PIN = 10;
int COLOR_RANGE = 384;

void setup() {
  //colorSetup();
  accelSetup();
  Serial.begin(9600);
}

void loop() {
  //showColors();
  pollAccel(50);
}


///////////
// accel //
///////////

Adafruit_LSM303 lsm;

void accelSetup() {
  lsm.begin();
}

void pollAccel(int delayMilliseconds) {
  lsm.read();
  int x = lsm.accelData.x;
  int y = lsm.accelData.y;
  int z = lsm.accelData.z;
  
  Serial.print(x); Serial.print (", ");
  Serial.print(y); Serial.print (", ");
  Serial.print(z);
  Serial.println();
  
  delay(delayMilliseconds);
}


///////////
// color //
///////////

LPD8806 strip = LPD8806(LED_COUNT, DATA_PIN, CLOCK_PIN);

void colorSetup() {
  // Turn the strip on.
  strip.begin();
  
  // Refresh the strip.
  strip.show();
}

void showColors() {
  for (int j = 0; j < 384; j++) {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, color(j, 0.5));
    }
    strip.show();
    delay(1);
  }
  
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
  delay(1);
}

// Color, 1 from 384.
uint32_t color(uint16_t color, float brightness)  {
  byte r, g, b;
  int range = color / 128;
  switch(range) {
    case 0: // Red to Yellow (1 to 128)
      r = 127 - color % 128;
      g = color % 128;
      b = 0;
      break;
    case 1: // Yellow to Teal (129 to 256)
      r = 0;
      g = 127 - color % 128;
      b = color % 128;
      break;
    case 2: // Teal to Purple (257 to 384)
      r = color % 128;
      g = 0;
      b = 127 - color % 128;
      break;
  }
  r *= brightness;
  g *= brightness;
  b *= brightness;
  return(strip.Color(r, g, b));
}

