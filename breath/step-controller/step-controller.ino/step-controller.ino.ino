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

int max_speed = 160000;

void setup() {
  // initialize serial communications at 9600 bps:
  motor.setMaxSpeed(max_speed);         // stp/s
  //motor.setPullInSpeed(max_speed);         // stp/s
  motor.setAcceleration(300000);    // stp/s^2
  Serial.begin(115200);

}

char readstr[32];
int strptr = 0;
int newpos = 0;
int oldpos = 0;

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
  controller.move(motor);
  motor.setTargetAbs(newpos);  // Set target position to 1000 steps from current position
  if (!controller.isRunning() &&  0) {
    motor.setMaxSpeed(max_speed);         // stp/s

    //controller.moveAsync(motor);
    Serial.println(newpos);
  }
  //Serial.print(outputValue);
  //Serial.print("\n");
  //Serial.println(outputValue);

}
