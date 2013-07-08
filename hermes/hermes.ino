#include "LPD8806.h"
#include "SPI.h"

int LED_COUNT = 16;
int DATA_PIN = 2;
int CLOCK_PIN = 3;
int COLOR_RANGE = 384;

LPD8806 strip = LPD8806(LED_COUNT, DATA_PIN, CLOCK_PIN);

void setup() {
  // Turn the strip on.
  strip.begin();
  
  // Refresh the strip.
  strip.show();
  
  Serial.begin(9600);
}

void loop() {
  for (int j = 0; j < 384; j++) {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, color(j));
    }
    strip.show();
    delay(10);
  }
  
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
  delay(1000);
}

// Color, 1 from 384.
uint32_t color(uint16_t color)  {
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
  return(strip.Color(r, g, b));
}

