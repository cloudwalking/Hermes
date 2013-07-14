// LED imports
#include "LPD8806.h"
#include "SPI.h"

// Accel imports
#include "Adafruit_LSM303.h"
#include "Wire.h"

#include "AccelReading.h"

void setup() {
  Serial.begin(9600);
  
  //colorSetup();
  
  accelSetup();
}

void loop() {
  //showColors();
  
  accelPoll(25);
}


///////////
// accel //
///////////

Adafruit_LSM303 lsm;
AccelReading accelBuffer[10];
int bufferPosition;

void accelSetup() {
  Serial.println("BEGIN");
  
  lsm.begin();
  
  bufferPosition = 0;

  for (int i = 0; i < bufferSize(); i++) {
    accelBuffer[i].x = 0;
    accelBuffer[i].y = 0;
    accelBuffer[i].z = 0;
  }
  
  printBuffer();
}

void accelPoll(int delayMilliseconds) {
  lsm.read();
  
  int newX = lsm.accelData.x;
  int newY = lsm.accelData.y;
  int newZ = lsm.accelData.z;
  
  AccelReading previousReading = getPreviousReading();
  AccelReading *currentReading = &accelBuffer[bufferPosition];

  currentReading->x = newX;
  currentReading->y = newY;
  currentReading->z = newZ;
  
  if (equalReadings(previousReading, *currentReading)) {
    delay(delayMilliseconds);
    return;
  }
  
  printBuffer();
  
  if (++bufferPosition >= bufferSize()) {
    bufferPosition = 0;
  }
  
  delay(delayMilliseconds);
}

int bufferSize() {
  return sizeof(accelBuffer) / sizeof(accelBuffer[0]);
}

void printBuffer() {
  Serial.print(accelBuffer[bufferPosition].x); Serial.print (", ");
  Serial.print(accelBuffer[bufferPosition].y); Serial.print (", ");
  Serial.print(accelBuffer[bufferPosition].z);
  Serial.println();
}

AccelReading getPreviousReading() {
  int previous = bufferPosition - 1;
  if (previous < 0) previous = bufferSize() - 1;
  return accelBuffer[previous];
}

bool equalReadings(AccelReading a, AccelReading b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}


///////////
// color //
///////////

int LED_COUNT = 16;
int DATA_PIN = 9;
int CLOCK_PIN = 10;
int COLOR_RANGE = 384;

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

