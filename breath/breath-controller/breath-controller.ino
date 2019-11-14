/*

  Breath controller -- move motor based on serial input commands
  uses serial input to control stepper motor output
*/


/* Teensy 3.2 pinout:

    Pin 2 Stepper count
    Pin 3 Stepper  direction

    Pin 6 - Neopixel data for both rings

    Pin 10 - Sensor enable switch: low to enable motion
    Pin 11 - LOW (ground for sensor enable)

    Pin 13 - onboard LED
    Pin 14 - Smoke sensor input

    Pin 22 - auto/sensor toggle, active low
    Pin 23 - limit switch, active low

*/

// Can't use fastLED for 4-color (RGBW) neopixels
#include <Adafruit_NeoPixel.h>

#define NEO_PIN 6
#define NUM_LEDS 12
#define BRIGHTNESS 70

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
#define MOTOR_MIN 100
#define MOTOR_RANGE (PPR/2)

// for automatic breathing
// was 500, 2500
#define MIN_TICKS 100
#define MAX_TICKS 2100

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

// Pin definitions
#define LIMIT_SW 23
#define RUN_SW 22
#define SENSOR_ENABLE 10
#define ENABLE_LOW 11

#define UP_BUTTON 21
#define DN_BUTTON 20
#define SMOKE_ALARM 14
#define ONBOARD_LED 13

Bounce limit = Bounce(LIMIT_SW, 20);
Bounce run_sw = Bounce(RUN_SW, 20);
Bounce sensor_enable = Bounce(SENSOR_ENABLE, 20);
Bounce up_button = Bounce(UP_BUTTON, 20);
Bounce dn_button = Bounce(DN_BUTTON, 20);


void setup() {
  int count = 0;
  // setup limit switch with pullup
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(LIMIT_SW, INPUT_PULLUP);
  pinMode(RUN_SW, INPUT_PULLUP);
  pinMode(SENSOR_ENABLE, INPUT_PULLUP);
  pinMode(ENABLE_LOW, OUTPUT);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DN_BUTTON, INPUT_PULLUP);
  pinMode(SMOKE_ALARM, INPUT);

  // This output pin acts as ground for other side of
  // sensor enable
  digitalWrite(ENABLE_LOW, LOW);

  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  Serial.println("breath-controller.ino 1/10/19 J Foote (final?)");
  delay(1000);

  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  newpos = 0;

  motor.setPullInSpeed(0);
  motor.setMaxSpeed(-PPR / 8);
  controller.rotateAsync(motor);

  while (limit.fallingEdge() == 0 )  {
    //if (0) {
    limit.update();
    show_one((uint8_t)(count++));
    //show_white((uint8_t)(newpos >> 4));
    Serial.print("Searching for limit at ");
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


  
  // Turn off led if we've turned it on for status
  digitalWrite(ONBOARD_LED, LOW);

  // Always read serial port even if we ignore the commands
  update_serial();

  if (run_sw.read()) {
    // automatic mode, transition to next state if needed
    update_state();
  }
  else {
    // sensor mode, move to position based on data
    update_sensor_servo();
  }

  if (digitalRead(SMOKE_ALARM) == HIGH) {
    smoke_value = 1.0;
  }
  else {
    // fade red to black slowly
    smoke_value =  0.9999 * smoke_value;
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

void breathe_led_float(float val) {
  val = 1.0 - val;
  // 0 <= val < 1.0
  // simulate breathing by lighting up progressive brightness and number of leds
  int red_val = gmap[int(255.0 * smoke_value)];

  Serial.println(smoke_value);

  // light up a fraction of the brightness
  int rgbright = gmap[int(255 * val)];
  // make blue a little dimmer so light is a little warmer
  int bbright = gmap[int(255 * val * 0.7)];
  int bar = int(strip.numPixels() * val);
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (  (i < bar) || (i >= (strip.numPixels() - bar))) {
      strip.setPixelColor(i, strip.Color(rgbright, rgbright, bbright ) );
    }
    else {
      strip.setPixelColor(i, strip.Color(red_val, 0,  0 ) );
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



// Read serial port and update newpos from any complete command received. 
void update_serial() {
  if (Serial.available() > 0) {
    // Indicate we have incoming serial data
    digitalWrite(ONBOARD_LED, HIGH);
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
}

void update_sensor_servo() {

  sensor_enable.update();


  if (sensor_enable.read() == 0) {
    // Don't do any motion if enable switch is off
    controller.stop();
    return;
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
