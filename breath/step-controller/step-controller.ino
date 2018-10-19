/*

  stepper controller -- move motor based on serial input commands
  use analog input to control stepper motor output
*/

#define MIN_TICKS 0
#define MAX_TICKS 3200



#include <Adafruit_NeoPixel.h>

#define NEO_PIN 6

#define PIN 6

#define NUM_LEDS 12

#define BRIGHTNESS 50


Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEO_PIN, NEO_GRB + NEO_KHZ800);


byte neopix_gamma[] = {
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

// 1600 ppr, so 1/2 revolution is 800 steps
#define MOTOR_MIN 300
#define MOTOR_RANGE (PPR/2)


char readstr[32];
int strptr = 0;
int newpos = 0;
int oldpos = 0;
int pos = 0;

int max_speed = 320000;
int hunt_speed = -50000;
int max_accel = 3000000;


//int run_accel =  1* PPR;
//int run_speed = 1* PPR;


int run_speed = 320000;
int run_accel = 2*PPR;


// Start at maxium low
float phase = 3 * PI / 2;

#define LIMIT_SW 23
#define RUN_SW 22
#define UP_BUTTON 21
#define DN_BUTTON 20

Bounce limit = Bounce(LIMIT_SW, 20);
Bounce run_sw = Bounce(RUN_SW, 20);
Bounce up_button = Bounce(UP_BUTTON, 20);
Bounce dn_button = Bounce(DN_BUTTON, 20);

int slew = 1;
int r_speed = 6;

void setup() {
  int count = 0;
  // setup limit switch with pullup
  pinMode(LIMIT_SW, INPUT_PULLUP);
  pinMode(RUN_SW, INPUT_PULLUP);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DN_BUTTON, INPUT_PULLUP);

  // initialize serial communications at 9600 bps:
  Serial.begin(115200);

  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  newpos = 0;
  // on power up, find limit switch
  motor.setMaxSpeed(hunt_speed);         // stp/s
  motor.setAcceleration(1000000);    // stp/s^2
  limit.update();
  while (limit.fallingEdge() == 0 ) {
    limit.update();
    newpos = newpos + 10;
    motor.setTargetAbs(newpos);
    show_one((uint8_t)(count++));
    //show_white((uint8_t)(newpos >> 4));
    controller.move(motor);
    Serial.print("Searching for limit at ");
    Serial.println(newpos);
    delay(2);
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

  motor.setMaxSpeed(run_speed);         // stp/s
  //motor.setPullInSpeed(max_speed + 1);         // stp/s
  motor.setAcceleration(run_accel);    // stp/s^2
}


void loop() {
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

      if (sensor_flag == 0) {
        // ease into new position
        sensor_flag = 1;
        motor.setMaxSpeed(hunt_speed);         // stp/s
        motor.setTargetAbs(newpos);
        controller.move(motor);
        motor.setMaxSpeed(run_speed);         // stp/s
      }

    }
    else {

      readstr[strptr++] = c; // continue adding to existing str
      readstr[strptr] = '\0';
    }
  }

  if ( newpos != oldpos) {

    if (!controller.isRunning()) {
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

void show_white(uint8_t val) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, neopix_gamma[val] ) );
  }
  strip.show();
}
void show_one(uint8_t it) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0) );
  }
  //it = it & 0x0F;
  //if (it < strip.numPixels()) {
  strip.setPixelColor(it % strip.numPixels(), strip.Color(0, 0, 0, neopix_gamma[128] ) );
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
      strip.setPixelColor(i, strip.Color(0, 0, 0, neopix_gamma[128] ) );
    } else {
      strip.setPixelColor(i, strip.Color(0, 0, 0, 0) );
    }
  }
  strip.show();
}
