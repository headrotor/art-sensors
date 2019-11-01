Arduino code to read data from CONTEC CMS50+ SpO2 Meter

Data from the sensor is in TTL serial format—despite the use of a Micro-B USB connector >:(    

Besides instantaneous pulse, I can read the SpO2 measurement (blood oxygen saturation) but it does not change fast enough to be entertaining :)

Hardware: Teensy 3.2 and Contec CMS50D+ SpO2 sensor

Software: https://github.com/headrotor/art-sensors/tree/master/spO2/spo2-demo
Reverse-engineered serial protocol is (mostly) is documented in software comments

