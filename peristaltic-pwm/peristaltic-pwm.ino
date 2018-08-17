/*
  breathing hacked by jtf
  for peristaltic pump motor driven with L298N board
 
*/

int ledPin = 13;    // LED connected to digital pin 9

const int m1_dir = 2; // motor 1 pin 1
const int m1_pwm = 3; // motor 1 pin 2, must be PWM pin

const int min_speed = 50;
const int max_speed = 250;


void set_speed(int mspeed) {
  if (mspeed < 0 ) {
    mspeed = abs(mspeed);
    if (mspeed > 255)
      mspeed = 255;
    analogWrite(m1_pwm, (255 - mspeed));
    digitalWrite(m1_dir, HIGH);
    return;
  }
  if (mspeed >= 0) {
    if (mspeed > 255)
      mspeed = 255;
    analogWrite(m1_pwm, mspeed);
    digitalWrite(m1_dir, LOW);
    return;
  }
  // speed is 0, come to a complete stop
  //digitalWrite(m1_pwm, LOW);
  //digitalWrite(m1_dir, LOW);

}

void setup() {
  //analogWriteFrequency(outPin, 2000);
  //analogWriteResolution(8);
  analogWriteFrequency(m1_pwm, 16000);
  //pinMode(m1_p, OUTPUT);
  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_dir, OUTPUT);
  pinMode(ledPin, OUTPUT);

}

void loop() {
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = min_speed ; fadeValue <= max_speed; fadeValue += 5) {
    set_speed(fadeValue);
    digitalWrite(ledPin, LOW);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
  delay(200);
  for (int fadeValue = max_speed; fadeValue >= min_speed ; fadeValue -= 5) {
    digitalWrite(ledPin, HIGH);    
    set_speed(fadeValue);
    delay(30);
  }
  set_speed(0);
  delay(200);

}
