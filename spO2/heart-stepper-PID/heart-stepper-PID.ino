//# reverse-engineered interface to read streaming data from
//# CONTEC Pulse Oximeter Model model CMS50D+
//
//# Note USB-serial converter is in the CABLE: micro-B connector on device is serial, NOT USB.
//
//### WARNING: UNFIT FOR ANY MEDICAL USE INCLUDING BUT NOT LIMITED TO DIAGNOSIS AND MONITORING
//### NO WARRANTY EXPRESS OR IMPLIED, UNDOCUMENTED SERIAL INTERFACE: USE AT OWN RISK
//
//# communication is over the USB-serial converter at 115200 baud, 8 bits, no parity or handshake.
//# Serial protocol seems to be mostly 9-byte packets.
//
//# Commands are all 9 bytes starting with  0x7D 0x81.
//# The third byte seems to be some kind of opcode.
//# Remaining bytes are padded with 0x80 although there may be data in a command I have not captured.
//# In the response, the high bit is set on all but the first byte.
//
//# the microusb connector is NOT USB it is serial!
//# Pinout (flat side up, looking INTO device female jack)
//
//# _____________
//# | 1 2 3 4 5 |
//# \._________./
//#
//# Pin 2: device RX (Host TX) USB white -- connect to Teensy pin 2
//# Pin 3: device TX (Host RX) USB green -- connect to Teensy pin 1
//# Pin 5: GND                 (black)
//# other pins: NC?


// Usage: send cmd1 to start streaming.

/*
         This is streaming data,
         first byte is 0x01, second is 0xEn where n indicates data validity:

         0xE0 valid real-time and pulse data (takes a few seconds, not shown in trace above)
         0xE1 real-time  data is valid, pulse and spo2 not valid
         0xE2 no valid data

         next two bytes are real-time pulse data (for graph) (with high bit set?)
         (unsure why two different values, but may echo bar graph and plot on device OLED.
         next two bytes are pulse in BPM, and spO2 percent (high bit is set so AND with 0x7F to get integer values)
         Last two bytes are 0xFFw

*/



/* Teensy 3.2 pinout:

    Pin 2 Squeeze stepper count (pulse)
    Pin 3 Squeeze stepper  direction

    Pin 4 Blow stepper count (pulse)
    Pin 5 Blow stepper  direction

    Pin 6 - Neopixel data out
    Pin 12 - RUN switch, active low for motion
    Pin 13 - on-board LED



    Pin 22 - blow limit switch, active low
    Pin 23 - squeeze limit switch, active low

*/

#include <StepControl.h>
#include <Bounce.h>

#include "FastLED.h"


// --------------------- set up FastLED library ------------
FASTLED_USING_NAMESPACE


#define DATA_PIN    6
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    24
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120


// --------------------- set up TeensyStepper library ------------
// (library from here: https://github.com/luni64/TeensyStep

#define BLOW_PPR (3200)
#define SQUEEZE_PPR (3200)


// minimum and range of motion, in motor ticks
#define BLOW_MIN 300
#define BLOW_RANGE (BLOW_PPR/3)
// Gain of PID loop (arbitrary units)
#define BLOW_GAIN (100)

#define SQUEEZE_MIN 300
#define SQUEEZE_RANGE (SQUEEZE_PPR/3)
#define SQUEEZE_GAIN (100)

// motor scale MOTOR_MIN < motor pos < MOTOR_MIN + MOTOR_RANGE
// negative scale for motion in opposite direction

// set up stepper motor controller
Stepper squeeze(4, 5);         // STEP pin: 2, DIR pin: 3
Stepper blow(2, 3);         // STEP pin: 2, DIR pin: 3
//Stepper blow(4, 5);         // STEP pin: 2, DIR pin: 3

StepControl<> controller;    // Use default settings

int newpos;
int oldpos;

//------------------------------------------------------ set up serial input from pulse oximeter----------------

// Commands to start streaming data from
uint8_t cmd1[] = {0x7D, 0x81, 0xA1, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80};
uint8_t cmd2[] = {0x7D, 0x81, 0xAF, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80};

// all communication is in 9-byte packets
#define PACKET_LEN 9

uint8_t inp[9];   // input buffer, read 9 bytes at a time
int led = 13;

// global variables hold state from last serial packet
int pulse  = 0;  // integer bpm
int pstatus = 0; // Status byte,
int beat = 0; // heartbeat amplitude for graph
int spo2 = 0;  // SpO2 value
int hphase = 0; // hearbeat phase, don't quite understand this, bargraph shown at right of display?

// Set up input switches ---------------------------------------------------------------------------------------------
// limit switches for each bellows
#define BLOW_LIMIT 22
#define SQUEEZE_LIMIT 23

// limit swictehs to find extremes of motor movement
Bounce blow_limit = Bounce(BLOW_LIMIT, 20);
Bounce squeeze_limit = Bounce(SQUEEZE_LIMIT, 20);

#define RUN_SW 12
#define RUN_LOW 11

// Switch between attract mode and sensor mode
Bounce run_sw = Bounce(RUN_SW, 20);

// ticks per degree
int tpd = 0;

void setup()   {
  int stepcount = 0;

  tpd = (int) (BLOW_PPR / (float) 360.0);

  Serial1.begin(115200); // hardware serial to device on gpio 0 and 1
  Serial.begin(115200);  // usb serial to host for debug
  pinMode(led, OUTPUT); // light LED on serial input from pulse meter
  pinMode(BLOW_LIMIT, INPUT_PULLUP);
  pinMode(SQUEEZE_LIMIT, INPUT_PULLUP);
  pinMode(RUN_SW, INPUT_PULLUP);
  pinMode(RUN_LOW, OUTPUT);
  // virtual ground for run switch (running out of ground pins)
  digitalWrite(RUN_LOW, LOW);


  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  newpos = 0;
  // on power up, hunt for limit switches
  blow.setPullInSpeed(BLOW_PPR / 8);
  blow.setMaxSpeed(BLOW_PPR / 8);
  controller.rotateAsync(blow);
  blow_limit.update();
  squeeze_limit.update();

  int homed_motors = 0;
  while ((squeeze_limit.fallingEdge() == 0) || (squeeze_limit.read() == HIGH)  ) {
    blow_limit.update();
    squeeze_limit.update();
    //newpos = newpos + 2*tpd;
    //motor.setTargetAbs(newpos);
    show_three((uint8_t)(stepcount++));
    //show_white((uint8_t)(newpos >> 4));
    //controller.move(motor);
    Serial.print("Searching for limit at ");
    Serial.println(newpos);
    delay(20);
  }
  controller.stop();

  show_all(CRGB::Black);
  Serial.print("Found  limit at ");
  Serial.println(newpos);
  // OK, found the limit switch, set this as zero
  newpos = BLOW_MIN;
  oldpos = BLOW_MIN;
  // Set this position to zero on the motor
  blow.setPosition(newpos);
  blow.setTargetAbs(newpos);
  controller.move(blow);

}

int oldbeat = 0; // detect changes in heartbeat position

int stepcount = 0;

void loop() {

  run_sw.update();

  if (run_sw.fallingEdge()) {
    // set velocity to zero
    controller.stop();
  }

  if (run_sw.read() == HIGH) {
    //if (run_sw.read() == HIGH) {
    //show_all(0, 0, 0);
    attract_loop_blow();
  }
  else {
    //data_loop_async();
    data_loop_PID();

    // insert a delay to keep the framerate modest
    //FastLED.delay(1000 / FRAMES_PER_SECOND);
  }
  fadeToBlackBy( leds, NUM_LEDS, 20);
  FastLED.show();
}


const int attract_speed = 3 * BLOW_PPR;

#define SYSTOLIC 1
#define DIASTOLIC 0

int attract_state = SYSTOLIC;


// attraction phase
// phase for blow motor
static float bphase = 0.;

//phase for squeeze motor
static float sphase = 0.;


void attract_loop_blow() {

  bphase = bphase + 0.01;
  float ftarget = 0.5 * (1 - cos(bphase));
  //graph_led_float(mpos);
  int target = (int)(BLOW_RANGE * ftarget) + BLOW_MIN;
  Serial.println(target);

  int diff = target - (int)blow.getPosition();

  //if ( newpos != oldpos ) {
  if (abs(diff) > 1)  {
    // quasi PID: set rotation speed to difference (error signal)
    //int off  = 100 * diff;


    // PID calculation, move with velocity proportional to offset
    int off = BLOW_GAIN * diff;

    blow.setPullInSpeed(off);
    blow.setMaxSpeed(off);
    //blow.setAcceleration(3200000);
    controller.rotateAsync(blow);
    Serial.print("move v");
    Serial.println(off);

    oldpos = newpos;
  }
  // Do we need to delay?
  delay(2);
}


int count = 999;


/*
 * void data_loop_async()
{
  // Do this to start serial streaming. Don't need it that often tho
  if (count > 100) {
    count = 0;
    Serial1.write((uint8_t *) &cmd1, sizeof(cmd1));
  }
  count++;


  digitalWrite(led, LOW);
  while (Serial1.available()) {
    digitalWrite(led, HIGH);
    parse_byte(Serial1.read());
  }

  // if heartbeat position has changed, update motor position
  if (oldbeat != beat)  {
    //motor.setMaxSpeed(1 * PPR);       // stp/s
    //motor.setAcceleration(1 * PPR);  // stp/s^2
    float pos = map_beat_float(beat);
    Serial.println(pos);


    float mpos = (squeeze.getPosition() - SQUEEZE_MIN) / (float)SQUEEZE_RANGE;
    graph_led_float(mpos);
    // PID calculation, move with velocity proportional to offset

    

    if (!controller.isRunning()) {
      //motor.setTargetAbs((int)(pos * (MOTOR_RANGE - (15 * tpd))) + MOTOR_MIN);
      //controller.moveAsync(motor);
    }
    oldbeat = beat;
  }

  FastLED.delay(10);
}
*/
int timeout = 0;

void data_loop_PID()
{
  // Do this to start serial streaming. Don't need it that often tho
  if (count > 100) {
    count = 0;
    Serial1.write((uint8_t *) &cmd1, sizeof(cmd1));
  }
  count++;
  timeout++;

  if (timeout == 100) {
    Serial.println("timeout!");
    controller.stop();
  }

  digitalWrite(led, LOW);
  while (Serial1.available()) {
    digitalWrite(led, HIGH);
    parse_byte(Serial1.read());
  }

  // if heartbeat position has changed, update motor position
  if (oldbeat != beat)  {
    timeout = 0;
    float pos = map_beat_float(beat);
    Serial.println(pos);

    graph_led_float(pos);

    int target = (int)(BLOW_RANGE * pos) + BLOW_MIN;
    int diff = target - (int)blow.getPosition();
    Serial.println(target);

    int off = 10 * diff;

    blow.setPullInSpeed(off);
    blow.setMaxSpeed(off);
    //blow.setAcceleration(3200000);
    controller.rotateAsync(blow);
    Serial.print("move b");
    Serial.println(off);



    if (!controller.isRunning()) {
      //motor.setTargetAbs((int)(pos * (MOTOR_RANGE - (15 * tpd))) + MOTOR_MIN);
      //controller.moveAsync(motor);
    }
    oldbeat = beat;
  }

  //FastLED.delay(10);
}

/*
    Parse the incomoing byte stream. Values are left in global variables pulse, spo2, and beat
    pstatus is refreshed every

*/

int bcount = -1; // byte count in this string
uint8_t cstr[9]; // accumulate command in this buffer

void parse_byte(uint8_t b) {
  // get a new byte, add it to command string
  if (b == 0x01) {
    // start of 9-byte packet, so reset count//
    bcount = 0;
  }
  if (bcount >= 0) {
    cstr[bcount++] = b & 0x7f; // clear high order bit of data
    if (bcount >= 9 ) {
      handle_packet();
      bcount = -1; // between packets, wait for next packet start
    }
  }
}

float map_beat_float(int beat) {
  // map integer heartbeat range into 0-1 float
  int bmin = 16;
  int bmax = 86;

  if (beat < bmin) {
    return (0.0);

  }
  if (beat > bmax) {
    return (1.0);
  }

  // return floating point beat value 0.0 < val < 1.0
  return ( (float) (beat - bmin) / (bmax - bmin));

}

void print_data() {
  Serial.print("Status: ");
  Serial.print(pstatus, HEX);
  Serial.print(" Pulse: ");
  Serial.print(pulse);
  Serial.print(" Beat: ");
  Serial.print(beat);
  Serial.print(" phase: ");
  Serial.print(hphase);
  Serial.print(" SpO2: ");
  Serial.print(spo2);
  Serial.println("");
}

static int bmin = 999999;
static int bmax = 0;

void analyse_data() {

  if ( beat < bmin) {
    bmin = beat;
  }
  if ( beat > bmax) {
    bmax = beat;
  }

  Serial.print(" Beat: ");
  Serial.print(beat);
  Serial.print(" min: ");
  Serial.print(bmin);
  Serial.print(" max: ");
  Serial.println(bmax);


}

void handle_packet() {

  // Complete packet just recieved; deal with it
  pstatus = cstr[1];
  beat = cstr[3];
  pulse = cstr[5];
  spo2 = cstr[6];
  hphase = cstr[4];

  for (int i = 0; i < 9; i++) {
    inp[i] = Serial1.read();
    //Serial.print(cstr[i], HEX);
    //Serial.print(" ");
  }
  //Serial.println(">");

  //analyse_data();
  //print_data();
}



void show_three(uint8_t it) {
  fadeToBlackBy( leds, NUM_LEDS, 60);
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    ;
  }

  leds[(it % NUM_LEDS)] += CRGB::Green;
  leds[((it + 8) % NUM_LEDS)] += CRGB::Blue;
  leds[((it + 16) % NUM_LEDS)] += CRGB::Cyan;

  FastLED.show();
}


void show_all( CRGB color) {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }

  FastLED.show();
}

void graph_led_float(float val) {
  int n = (int) (val * (NUM_LEDS));
  int i = 0;
  //Serial.println(n);
  for (i = 0; i < n; i++) {
    leds[i].red += 20;
  }
  for (; i < NUM_LEDS; i++) {
    //leds[i] = CRGB::Black;
  }
}

void show_all(uint8_t r, uint8_t g, uint8_t b ) {

}


/* for adafruit neopixel
  void show_one(uint8_t it) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0) );
  }
  //it = it & 0x0F;
  //if (it < strip.numPixels()) {
  strip.setPixelColor(it % strip.numPixels(), strip.Color(gmap[128], gmap[128], gmap[128] ) );
  //}
  strip.show();
  }

  void show_three(uint8_t it) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0) );
  }
  //it = it & 0x0F;
  //if (it < strip.numPixels()) {
  strip.setPixelColor(it % strip.numPixels(), strip.Color(gmap[128], gmap[128], gmap[128] ) );
  strip.setPixelColor((it + 8) % strip.numPixels(), strip.Color(gmap[128], gmap[128], gmap[128] ) );
  strip.setPixelColor((it + 16) % strip.numPixels(), strip.Color(gmap[128], gmap[128], gmap[128] ) );
  //}
  strip.show();
  }

  void graph_led_float(float val) {

  // 0 <= val < 1.0
  // bargraph

  // light up a fraction of the brightness
  int bright = gmap[int(255 * val)];
  int bar = int(strip.numPixels() * val);
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (  (i < bar) || (i >= (strip.numPixels() - bar))) {
      strip.setPixelColor(i, strip.Color(bright, bright, bright ) );
    }
    else {
      strip.setPixelColor(i, strip.Color(0, 0, 0 ) );
    }
  }

  strip.show();
  }

  void show_all(uint8_t r, uint8_t g, uint8_t b ) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b, 0) );
  }
  strip.show();
  }
*/
