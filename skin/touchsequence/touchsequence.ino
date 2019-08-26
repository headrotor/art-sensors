/*********************************************************



**********************************************************/

#include <Wire.h>
#include "Adafruit_MPR121.h"
#include "FastLED.h"

#include "gamma.h"

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif


///////////////////////////////////////////////////////////////////////////////////////////
//
// Move a white dot along the strip of leds.  This program simply shows how to configure the leds,
// and then how to turn a single pixel white and then off, moving down the line of pixels.
//

// How many leds are in the strip?
#define NUM_LEDS 24

// Data pin that led data will be written out over
#define DATA_PIN 6

// PWM white LEDs on this pin
# define WHITE_LEDS 4

// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];


uint8_t attr_phase = 0;



// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;



// state definitions

#define WAIT 0
#define TRIG 1

static uint8_t lr_state = 0;
static uint8_t rl_state = 0;

#define NCHAN 4 // number of channels used


static uint16_t red_val = 0;
static uint16_t blu_val = 0;

void lr_trig(void) {
  Serial.println("Right stroke!");
  for (int pos = 0; pos < NUM_LEDS; pos++) {
    leds[pos] += CRGB(0, 0, 32);
  }
}

void rl_trig(void) {
  Serial.println("Left stroke!");

  for (int pos = 0; pos < NUM_LEDS; pos++) {
    leds[pos] += CRGB(0,32, 0);
  }


}


void lr_advance(void) {
  lr_state += 1;
  if (lr_state >= TRIG) {
    lr_trig();
    lr_state = 0;
  }
  //Serial.print("new LR state " );
  //Serial.println(lr_state);
}

void rl_advance(void) {
  rl_state += 1;
  if (rl_state >= TRIG) {
    rl_trig();
    rl_state = 0;
  }
  //Serial.print("new RL state " );
  //Serial.println(rl_state);
}


void setup() {
  Serial.begin(9600);

  while (!Serial) { // needed to keep leonardo/micro from starting too fast!
    delay(10);
  }

  pinMode(WHITE_LEDS, OUTPUT);
  analogWriteFrequency(WHITE_LEDS, 12000);
  analogWrite(WHITE_LEDS, 0);

  Serial.println("Adafruit MPR121 Capacitive Touch sensor test");

  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring ? ");
    while (1);
  }
  Serial.println("MPR121 found!");

  cap.setThresholds(5, MPR121_RELEASE_THRESHOLD_DEFAULT );
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);

}


void loop() {
  // Get the currently touched pads
  uint8_t touchflag = 0;
  currtouched = cap.touched();

  for (uint8_t i = 0; i < NCHAN; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      //Serial.print(i); Serial.println(" touched");
      touchflag = 1;
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      //Serial.print(i); Serial.println(" released");
    }
  }

  if (touchflag) {
    touchflag = 0;
    //Serial.println(currtouched + 32, BIN);
    //Serial.println(lasttouched + 32, BIN);
    if (currtouched & (lasttouched << 1)) {
      lr_advance();
    }
    if (currtouched & (lasttouched >> 1)) {
      rl_advance();
    }

  }

  // reset our state
  lasttouched = currtouched;

  // comment out this line for detailed data from the sensor!
  print_values();
  fadeToBlackBy( leds, NUM_LEDS, 1);

  FastLED.delay(1000/30);
  FastLED.show();
  if (!leds[0].blue && !leds[0].red && !leds[0].green) {
    analogWrite(WHITE_LEDS, 150-0.5*cos8(attr_phase));
    
  }
  else {
    analogWrite(WHITE_LEDS, 0);
    attr_phase = 0;    
  }

  attr_phase += 2;

}

void print_values() {
  // debugging info, what
  Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t 0x"); Serial.println(cap.touched(), HEX);
  Serial.print("Filt : ");
  for (uint8_t i = 0; i < NCHAN; i++) {
    Serial.print(cap.filteredData(i)); Serial.print("\t");
  }
  Serial.print(currtouched + 32, BIN);
  Serial.println();
  Serial.print("Base : ");
  for (uint8_t i = 0; i < NCHAN; i++) {
    Serial.print(cap.baselineData(i)); Serial.print("\t");
  }
  Serial.println();
  // put a delay so it isn't overwhelming
  FastLED.delay(100);
}
