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


#include <StepControl.h>
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


void setup()   {

  Serial1.begin(115200); // hardware serial to device on gpio 0 and 1
  Serial.begin(115200);  // usb serial to host for debug
  pinMode(led, OUTPUT); // light LED on serial input from pulse meter
  pinMode(LIMIT_SW, INPUT_PULLUP);

  newpos = 0;
  // on power up, find limit switch
  motor.setMaxSpeed(hunt_speed);         // stp/s
  motor.setAcceleration(1000000);    // stp/s^2
  limit.update();
  while ((limit.fallingEdge() == 0) || (limit.read() == HIGH)  ) {
    limit.update();
    newpos = newpos + 150;
    motor.setTargetAbs(newpos);
    //show_one((uint8_t)(count++));
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


  // we are at the far negative extent so set the phase

  //newpos = neutral;
  //oldpos = neutral;
  motor.setTargetAbs(newpos);
  controller.move(motor);

  motor.setMaxSpeed(3000000);         // stp/s
  motor.setAcceleration(10000000);    // stp/s^2
}

int oldbeat = 0; // detect changes in heartbeat position

void loop() {
  //attract_loop();
  stepper_loop();

}

#define MIN_POS 0
#define MAX_POS 12800



const int attract_speed = 30000;
void attract_loop() {


  motor.setMaxSpeed(attract_speed);         // stp/s
  motor.setAcceleration( 2 * attract_speed); // stp/s^2

  motor.setTargetAbs(MIN_POS);
  //motor.setTargetAbs(1000*hphase); // This doesn't work so great.
  controller.move(motor);

  motor.setMaxSpeed(2 * attract_speed);       // stp/s
  motor.setAcceleration(5 * attract_speed); // stp/s^2

  motor.setTargetAbs(MAX_POS);
  //motor.setTargetAbs(1000*hphase); // This doesn't work so great.
  controller.move(motor);
}


int count = 999;

void stepper_loop()
{


  // Do this to start serial streaming. Don't need it that often tho
  if (count > 100) {
    count = 0;
    Serial1.write((uint8_t *) &cmd1, sizeof(cmd1));
  }
  count++;

  
  digitalWrite(led, LOW);
  //Serial1.flush();
  while (Serial1.available()) {
    digitalWrite(led, HIGH);
    parse_byte(Serial1.read());
  }

  // if heartbeat position has changed, update motor position
  if (oldbeat != beat) {
    motor.setTargetAbs(200 * beat);
    //motor.setTargetAbs(1000*hphase); // This doesn't work so great.
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

void handle_packet() {

  // Complete packet just recieved; deal with it
  pstatus = cstr[1];
  beat = cstr[3];
  pulse = cstr[5];
  spo2 = cstr[6];
  hphase = cstr[4];

  for (int i = 0; i < 9; i++) {
    inp[i] = Serial1.read();
    Serial.print(cstr[i], HEX);
    Serial.print(" ");
  }
  Serial.println(">");

  print_data();
}
