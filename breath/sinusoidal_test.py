from __future__ import print_function # WalabotAPI works on both Python 2 an 3.
import sys
import math
from sys import platform
from os import system
from imp import load_source
import time
import os
import serial


low_limit = 1600
high_limit = low_limit + 3200
# kind of the center point, return here at zero signal
offset = high_limit/3

# arbitrary numbers here, between 1 and 20?
gain = 15.
ser = serial.Serial('COM3', 115200, timeout=0)
    
def chartx(x, clen=70, c='#'):
    """generate a string of max length clen which is a bargraph of 
    floating point value 0. <= x <= 1 consisting of character c """
    if x > 1.:
        x = 1.
    slen = int(x*clen)
    cstr = [c for s in range(slen)]
    return "".join(cstr)

def TestingApp():
    phase = 0.0
    period = 0.03
    pinc = 0.5*math.pi*2*period
    while True:

        fpos = math.cos(phase)
        phase = phase + pinc


        pos = int((high_limit - low_limit)* (fpos + 1)/2.) + low_limit

        print(chartx(0.5*(fpos + 1)))
        sys.stdout.flush()
        if pos > high_limit:
            pos = high_limit
        if pos < low_limit:
           pos = low_limit;
           
        # send the position data to the stepper controller over the serial port
        ser.write("{:d}\n".format(pos).encode('utf-8'))
        ser.flushOutput()
        
        # display a bargraph
        #print("{0:6.2f} ".format(fpos))
        print("{:d} ".format(pos))
        time.sleep(period)


if __name__ == '__main__':
    TestingApp()
