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


#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define NEO_PIN 6



#define NUM_LEDS 24
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


#include <StepControl.h>

#define MOTOR_MIN 0
#define MOTOR_RANGE 12800



// motor scale MOTOR_MIN < motor pos < MOTOR_MIN + MOTOR_RANGE
// negative scale for motion in opposite direction


#include <Bounce.h>

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

// set up stepper motor controller
Stepper motor(2, 3);         // STEP pin: 2, DIR pin: 3
StepControl<> controller;    // Use default settings
int hunt_speed = -500000;
int newpos;
int oldpos;


#define LIMIT_SW 23

Bounce limit = Bounce(LIMIT_SW, 20);

#define RUN_SW 12
#define RUN_LOW 11

Bounce run_sw = Bounce(RUN_SW, 20);

void setup()   {
  int stepcount = 0;
  Serial1.begin(115200); // hardware serial to device on gpio 0 and 1
  Serial.begin(115200);  // usb serial to host for debug
  pinMode(led, OUTPUT); // light LED on serial input from pulse meter
  pinMode(LIMIT_SW, INPUT_PULLUP);
  pinMode(RUN_SW, INPUT_PULLUP);
  pinMode(RUN_LOW, OUTPUT);
  // virtual ground for run switch (running out of ground pins)
  digitalWrite(RUN_LOW, LOW);


  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  newpos = 0;
  // on power up, find limit switch
  motor.setMaxSpeed(hunt_speed);         // stp/s
  motor.setAcceleration(1000000);    // stp/s^2
  limit.update();
  while ((limit.fallingEdge() == 0) || (limit.read() == HIGH)  ) {
    limit.update();
    newpos = newpos + 150;
    motor.setTargetAbs(newpos);
    show_three((uint8_t)(stepcount++));
    //show_white((uint8_t)(newpos >> 4));
    controller.move(motor);
    Serial.print("Searching for limit at ");
    Serial.println(newpos);
    delay(2);
  }
  //show_all(0, 128, 0);
  Serial.print("Found  limit at ");
  Serial.println(newpos);
  // OK, found the limit switch, set this as zero
  newpos = 0;
  oldpos = 0;
  // Set this position to zero on the motor
  motor.setPosition(newpos);
  // back off a few degrees?

  motor.setTargetAbs(newpos);
  controller.move(motor);


}

int oldbeat = 0; // detect changes in heartbeat position

int stepcount = 0;

void loop() {

  run_sw.update();
  if (run_sw.read() == HIGH) {
    show_all(0,0,0);
    attract_loop();
  }
  else {
    data_loop();
  }
}


const int attract_speed = 50000;
void attract_loop() {


  motor.setMaxSpeed(attract_speed);         // stp/s
  motor.setAcceleration( 2 * attract_speed); // stp/s^2

  motor.setTargetAbs(MOTOR_MIN);
  controller.move(motor);

  motor.setMaxSpeed(2 * attract_speed);       // stp/s
  motor.setAcceleration(5 * attract_speed); // stp/s^2

  motor.setTargetAbs(MOTOR_MIN + MOTOR_RANGE);
  //motor.setTargetAbs(1000*hphase); // This doesn't work so great.
  controller.move(motor);
}


int count = 999;

void data_loop()
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
  if (oldbeat != beat) {
    motor.setMaxSpeed(3000000);         // stp/s
    motor.setAcceleration(10000000);    // stp/s^2
    float pos = map_beat_float(beat);
    //Serial.println(pos);
    graph_led_float(pos/2.);
    motor.setTargetAbs((int)(pos * MOTOR_RANGE) + MOTOR_MIN);
    controller.move(motor);
    oldbeat = beat;
  }

  delay(10);
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
