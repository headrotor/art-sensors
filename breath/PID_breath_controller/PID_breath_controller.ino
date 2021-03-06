/*

  stepper controller -- move motor based on serial input commands
  use analog input to control stepper motor output
*/


/* Teensy 3.2 pinout:

    Pin 2 Stepper count
    Pin 3 Stepper  direction

    Pin 6 - Neopixel data for both rings

    Pin 14 - Smoke sensor input

    Pin 22 - auto/sensor toggle, active low
    Pin 23 - limit switch, active low

*/

// Can't use fastLED for 4-color (RGBW) neopixels
#include <Adafruit_NeoPixel.h>

#define NEO_PIN 6
#define PIN 6
#define NUM_LEDS 12
#define BRIGHTNESS 50

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEO_PIN, NEO_GRB + NEO_KHZ800);

byte gmap[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
  10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
  17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
  25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
  37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
  51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
  69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
  90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
  115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
  177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
  215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
};

int sensor_flag = 0;

#define INHALE 0
#define INHALE_PAUSE 1
#define EXHALE 2
#define EXHALE_PAUSE 3

// state machine for inhale/exhale
int m_state = EXHALE_PAUSE;
elapsedMillis time_in_state;
float smoke_value = 0.0;

#include <StepControl.h>

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from sensor
int outputValue = 0;        // Output position value to motor
int analog_input_pin = 15;

Stepper motor(2, 3);         // STEP pin: 2, DIR pin: 3
StepControl<> controller;    // Use default settings
#include <Bounce.h>

#define PPR (4000)

// for sensor
#define MOTOR_MIN 300
#define MOTOR_RANGE (PPR/2)

// for automatic breathing
#define MIN_TICKS 500
#define MAX_TICKS 2500

int auto_speed = 3000;
int auto_accel = 500;

/*
  //First cut with slew limit
  int sensor_speed = 30 * PPR;
  int sensor_accel = 5 * PPR;

  // Limit motor position command to this
  int slew_limit = 50;
*/

// Todo: reset position with limit switch
// more careful serial flush

int sensor_speed = 10 * PPR;
int sensor_accel = 2 * PPR;

// Limit motor position command to this
int slew_limit = 2000;


// serial input commands go in this buffer
char readstr[32];
int strptr = 0;
int newpos = 0;
int oldpos = 0;
int pos = 0;


#define LIMIT_SW 23
#define RUN_SW 22
#define UP_BUTTON 21
#define DN_BUTTON 20

Bounce limit = Bounce(LIMIT_SW, 20);
Bounce run_sw = Bounce(RUN_SW, 20);
Bounce up_button = Bounce(UP_BUTTON, 20);
Bounce dn_button = Bounce(DN_BUTTON, 20);


void setup() {
  int count = 0;
  // setup limit switch with pullup
  pinMode(LIMIT_SW, INPUT_PULLUP);
  pinMode(RUN_SW, INPUT_PULLUP);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DN_BUTTON, INPUT_PULLUP);

  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  Serial.println("PID_breath_controller 1/10/19 J Foote");
  delay(1000);

  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  newpos = 0;
  // on power up, find limit switch
  limit.update();
  int ppd = PPR / 360; // pulses per degree
  
  motor.setPullInSpeed(0);
  motor.setMaxSpeed(-PPR / 8);
  controller.rotateAsync(motor);
  
  while (limit.fallingEdge() == 0 )  {
  //if (0) {
    limit.update();
    show_one((uint8_t)(count++));
    //show_white((uint8_t)(newpos >> 4));
    Serial.print("Searching for limit at ");
    Serial.println(newpos);
    delay(20);
  }
  show_all(0, 128, 0);
  Serial.print("Found  limit at ");
  Serial.println(newpos);
  // OK, found the limit switch, set this as zero
  newpos = 0;
  oldpos = 0;
  // Set this position to zero on the motor
  motor.setPosition(newpos);
  // back off a few degrees?


  // we are at the far negative extent so set the phase

  //newpos = neutral;
  //oldpos = neutral;
  motor.setTargetAbs(newpos);
  controller.move(motor);

}

void loop() {

  run_sw.update();
  up_button.update();
  dn_button.update();

  //  if (run_sw.read()) {
  if (0) {
    // automatic mode, transition to next state if needed
    update_state();
  }
  else {
    // sensor mode, move to position based on data
    update_sensor_servo();
  }
  breathe_led_float(get_motor_pos());

}


float get_motor_pos(void) {
  /// return motor position as a float between 0 and 1.
  int pos = motor.getPosition();
  int ext = MAX_TICKS - MIN_TICKS;
  return ( float(pos - MIN_TICKS) / float(ext) );
}

void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
  strptr = 0;

}
void update_state() {
  motor.setMaxSpeed(auto_speed);       // stp/s
  motor.setAcceleration(auto_accel);         // stp/s
  int automatic = 1;

  switch (m_state) {
    case INHALE:
      if (! controller.isRunning()) {
        // we've reached the end of the breath, change state
        //Serial.println("in pause");
        if (automatic) {
          m_state = INHALE_PAUSE;
        }
        time_in_state = 0;
      }
      break;

    case INHALE_PAUSE:
      if (time_in_state > 200) {
        Serial.println("exhale");
        m_state = EXHALE;
        motor.setTargetAbs(MAX_TICKS);
        controller.moveAsync(motor);
        delay(10);
      }
      break;


    case EXHALE:
      if (! controller.isRunning()) {
        // we've reached the end of the breath, change state
        //Serial.println("exhale pause");
        if (automatic) {
          m_state = EXHALE_PAUSE;
        }
        time_in_state = 0;
      }
      break;

    case EXHALE_PAUSE:
      //Serial.println(time_in_state);
      if (time_in_state > 100) {
        Serial.println("inhale");
        m_state = INHALE;
        motor.setTargetAbs(MIN_TICKS);
        controller.moveAsync(motor);
        delay(10);
      }
      break;
  }
  //Serial.print("Current state is ");
  //Serial.println(m_state);
}

void inhale_now() {
  // jumpstart state machine from external trigger
  if (m_state != EXHALE) {
    // ignore commands unless exhaling
    return;
  }
  Serial.println("inhaling now");
  m_state = EXHALE_PAUSE;
  time_in_state = 999;
  update_state();
}

void exhale_now() {
  // jumpstart state machine from external trigger
  if (m_state != INHALE) {
    // ignore commands unless inhaling
    return;
  }

  Serial.println("exhaling now");
  m_state = INHALE_PAUSE;
  time_in_state = 999;
  update_state();
}

int target_pos = MOTOR_MIN;

void update_sensor_limits() {
  //delay(3);  //delay to allow buffer to fill
  if (Serial.available() > 0) {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == '\n') {
      // newline; terminate string
      //Serial.println("got!");
      //Serial.println(readstr);
      newpos = atoi(readstr);
      //Serial.println(newpos);
      strptr = 0;
    }
    else {

      readstr[strptr++] = c; // continue adding to existing str
      readstr[strptr] = '\0';
    }
  }

  if ( newpos != oldpos) {

    if (!controller.isRunning()) {
      motor.setMaxSpeed(sensor_speed);       // stp/s
      motor.setAcceleration(sensor_accel);         // stp/s

      int pos = motor.getPosition();
      int diff =  newpos - pos ;
      int lim = slew_limit;
      if (abs(diff) > lim) {
        if (diff > 0) {
          newpos = pos + lim;
        }
        else if (diff < 0) {
          newpos = pos - lim;
        }
      }

      serialFlush();

      motor.setTargetAbs(newpos);  // new target position
      controller.moveAsync(motor);
      // rough maximum is 2000, map to 250
      // show_white((uint8_t)(newpos >> 3));
    }
    //Serial.println(newpos);

    bargraph_1((uint8_t)(newpos >> 7));
    oldpos = newpos;
  }
  delay(1);
}


void update_sensor() {
  //delay(3);  //delay to allow buffer to fill
  if (Serial.available() > 0) {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == '\n') {
      // newline; terminate string
      //Serial.println("got!");
      //Serial.println(readstr);
      newpos = atoi(readstr);
      //Serial.println(newpos);
      strptr = 0;
    }
    else {

      readstr[strptr++] = c; // continue adding to existing str
      readstr[strptr] = '\0';
    }
  }

  if ( newpos != oldpos) {

    if (!controller.isRunning()) {

      motor.setMaxSpeed(sensor_speed);       // stp/s
      motor.setAcceleration(sensor_accel);         // stp/s
      motor.setTargetAbs(newpos);  // new target position
      controller.moveAsync(motor);
      // rough maximum is 2000, map to 250
      // show_white((uint8_t)(newpos >> 3));
    }
    //Serial.println(newpos);

    bargraph_1((uint8_t)(newpos >> 7));
    oldpos = newpos;
  }
  delay(1);
}

void update_sensor_PID_OLD() {
  int diff = 0;
  if (Serial.available() > 0) {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == '\n') {
      // newline; terminate string
      //Serial.println("got!");
      //Serial.println(readstr);
      newpos = atoi(readstr);
      //Serial.println(newpos);
      strptr = 0;
    }
    else {

      readstr[strptr++] = c; // continue adding to existing str
      readstr[strptr] = '\0';
    }
  }


  diff = newpos - motor.getPosition();
  if ( newpos != oldpos ) {
    if (abs(diff) > 100)  {
      // quasi PID: set rotation speed to difference (error signal)
      int off  = 1000 * diff;
      motor.setMaxSpeed(off);
      motor.setAcceleration(320000);
      controller.rotateAsync(motor);
      Serial.print("move ");
      Serial.println(off);

      //serialFlush();
    }
  }


  if (abs(diff) < 100)  {
    motor.setMaxSpeed(0);
    controller.rotateAsync(motor);
  }


  oldpos = newpos;
  delay(5);
  Serial.println(diff);
}


void update_sensor_servo() {
  if (Serial.available() > 0) {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == '\n') {
      // newline; terminate string
      //Serial.println("got!");
      //Serial.println(readstr);
      newpos = atoi(readstr);
      //Serial.println(newpos);
      strptr = 0;
    }
    else {

      readstr[strptr++] = c; // continue adding to existing str
      readstr[strptr] = '\0';
    }
  }

  int diff = newpos - motor.getPosition();

  if (abs(diff) > 1)  {
    // quasi PID: set rotation speed to difference (error signal)
    // PID calculation, move with velocity proportional to offset
    int off = 10 * diff;

    motor.setPullInSpeed(off);
    motor.setMaxSpeed(off);
    //squeeze.setAcceleration(3200000);
    controller.rotateAsync(motor);
    //Serial.print("squeeze ");
    //Serial.println(off);
  }

}

void update_sensor_PID_WHAT() {
  int diff = 0;
  if (Serial.available() > 0) {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == '\n') {
      // newline; terminate string
      //Serial.println("got!");
      //Serial.println(readstr);
      newpos = atoi(readstr);
      //Serial.println(newpos);
      strptr = 0;
    }
    else {

      readstr[strptr++] = c; // continue adding to existing str
      readstr[strptr] = '\0';
    }
  }


  diff = newpos - motor.getPosition();
  if ( newpos != oldpos ) {
    //if (abs(diff) > 10)  {
    // quasi PID: set rotation speed to difference (error signal)
    //int off  = 100 * diff;
    int off  = 0;



    if (diff > 0) {
      off = 100000;
    }
    else {
      
      off = -100000;

    }

    off = 100 * diff;

    motor.setPullInSpeed(off);
    motor.setMaxSpeed(off);
    //motor.setAcceleration(3200000);
    controller.rotateAsync(motor);
    Serial.print("move ");
    Serial.println(off);
    //delay(10);
    //serialFlush();
    //}
    oldpos = newpos;
  }


  if (abs(diff) < 5)  {
    motor.setMaxSpeed(0);
    controller.rotateAsync(motor);
  }


  //delay(5);
  Serial.println(diff);
}


void breathe_led_float(float val) {
  val = 1.0 - val;
  // 0 <= val < 1.0
  // simulate breathing by lighting up progressive brightness and number of leds
  int red_val = gmap[int(255.0 * smoke_value)];

  // light up a fraction of the brightness
  int bright = gmap[int(255 * val)];
  int bar = int(strip.numPixels() * val);
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (  (i < bar) || (i >= (strip.numPixels() - bar))) {
      strip.setPixelColor(i, strip.Color(bright, bright, bright ) );
    }
    else {
      strip.setPixelColor(i, strip.Color(0, red_val, 0 ) );
    }
  }  // light up some fraction of the leds to full

  strip.show();
}


/************************************/

void show_white(uint8_t val) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, gmap[val] ) );
  }
  strip.show();
}
void show_one(uint8_t it) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0) );
  }
  //it = it & 0x0F;
  //if (it < strip.numPixels()) {
  strip.setPixelColor(it % strip.numPixels(), strip.Color(gmap[128], gmap[128], gmap[128], gmap[128] ) );
  //}
  strip.show();
}

void show_all(uint8_t r, uint8_t g, uint8_t b ) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b, 0) );
  }
  strip.show();
}

void bargraph_1(uint8_t val ) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (i < val) {
      strip.setPixelColor(i, strip.Color(0, 0, 0, gmap[128] ) );
    } else {
      strip.setPixelColor(i, strip.Color(0, 0, 0, 0) );
    }
  }
  strip.show();
}
