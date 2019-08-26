#!/usr/bin/env python


import time
import serial
import subprocess
import sys
# can't import breath.py because path bullshit so just exec it


def setled(ser, color):
    for i in range(1):
        #ser.write(b'F10\n')
        ser.write(color)     # turn LED green
        #while(ser.inWaiting):
        #    foo = ser.read(1)
        ser.flush()

try:
    ser = serial.Serial('COM4',19200,timeout=0.1)  # open serial port
except serial.SerialException as e:
    time.sleep(5)
    raise e
    
ser.write(b'F10\r\n')
setled(ser, b'#FF00FF\r\n')     # turn LED blue for startup

time.sleep(3)


count = 0
while(True):
    #process = subprocess.Popen("python C:\\kalbot\\gith\\art-sensors\\breath\\led.py")
    setled(ser, b'#00FF00\r\n');
    time.sleep(3)
    process = subprocess.Popen("python C:\\kalbot\\gith\\art-sensors\\breath\\breath.py")
    process.wait()
    setled(ser, b'#FF0000\r\n')     # turn LED red
    print("crashed, restarting")
    time.sleep(3)

"""
*******************************************************

P                       - Enter Device firmware update mode

?                       - Return device UUID #

#RRGGBB                 - Set LED color according to Hex color code

F                       - Set Fade transition Time in ms 'F1000'

G                       - Return current color, (rr,gg,bb)

B                       - Set Fade transition Colors B#RRGGBB-tttt#RRGGBB....

                         Fade transition after each color


Firmware Revision: V0.9.5

*******************************************************


def setled(color):
    ser = serial.Serial('COM4',19200,timeout=0)  # open serial port
    for i in range(5):
        ser.write(color)     # turn LED green
    #ser.flush()
    ser.close()


"""
