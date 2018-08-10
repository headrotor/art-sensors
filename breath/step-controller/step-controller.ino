/*

  stepper controller -- move motor based on serial input commands
  use analog input to control stepper motor output
*/


#include <StepControl.h>

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from sensor
int outputValue = 0;        // Output position value to motor
int analog_input_pin = 15;

Stepper motor(2, 3);         // STEP pin: 2, DIR pin: 3
StepControl<> controller;    // Use default settings
#include <Bounce.h>


char readstr[32];
int strptr = 0;
int newpos = 0;
int oldpos = 0;
int pos = 0;

int max_speed = 320000;

int max_accel = 3000000;
int hunt_speed = 4000;
int neutral = 750; // neutal offset from zero

#define LIMIT_SW 23
Bounce limit = Bounce(LIMIT_SW, 20);

int slew = 1;

void setup() {
  
  // setup limit switch with pullup
  pinMode(LIMIT_SW, INPUT_PULLUP);

  // initialize serial communications at 9600 bps:
  Serial.begin(115200);

  newpos = 0;
  // on power up, find limit switch
  motor.setMaxSpeed(hunt_speed);         // stp/s
  motor.setAcceleration(100000);    // stp/s^2
  limit.update();
  while (limit.fallingEdge() == 0 ) {
    limit.update();
    newpos = newpos + 4;
    motor.setTargetAbs(newpos);
    controller.move(motor);
    Serial.print("Searching for limit at ");
    Serial.println(newpos);
    //delay(2);
  }
  // OK, found the limit switch, set this as zero
  newpos = 0;
  oldpos = 0;
  pos = 0;
  motor.setPosition(newpos);
  // back off a few degrees?

  newpos = neutral;
  oldpos = neutral;
  motor.setTargetAbs(newpos);
  controller.move(motor);

  motor.setMaxSpeed(max_speed);         // stp/s
  //motor.setPullInSpeed(max_speed + 1);         // stp/s
  motor.setAcceleration(max_accel);    // stp/s^2

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
    }
    else {

      readstr[strptr++] = c; // continue adding to existing str
      readstr[strptr] = '\0';
    }
  }

  //sensorValue = analogRead(A1);
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, min_speed, max_speed);
  //motor.setMaxSpeed(outputValue);

  /*
    while (newpos != pos || 0) {
      if (pos > newpos) {
        pos -= slew;
      }
      else if (pos < newpos) {
        pos += slew;
      }
      motor.setTargetAbs(newpos);  // Set target position to 1000 steps from current position
      controller.move(motor);
    }
  */

  if ( newpos != oldpos) {
    motor.setTargetAbs(newpos);  // new target position
    controller.move(motor);
    oldpos = newpos;
  }

  //Serial.print(outputValue);
  //Serial.print("\n");
  //Serial.println(outputValue);

}
