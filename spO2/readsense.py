#!/usr/bin/python
# -*- coding: utf-8 -*-
# hardware interface to Alicat flow controllser


import serial
import sys
import time
import threading
"""

         7D 81 A7 80 80 80 80 80 80  0C 80 
         7D 81 A2 80 80 80 80 80 80  0C 80 
         7D 81 A0 80 80 80 80 80 80  06 80 80 87 
         7D 81 B0 80 80 80 80 80 80  11 80 81 81 80 80 80 80 80 
         7D 81 AC 80 80 80 80 80 80  0E 80 81 
         7D 81 B3 80 80 80 80 80 80  13 80 A0 A0 A0 A0 A0 A0 A0 14 80 A0 A0 A0 A0 A0 80 80  
         7D 81 A8 80 80 80 80 80 80  02 80 80 A0 A0 A0 A0 A0 A0 02 81 FF A0 A0 A0 A0 A0 A0 
         7D 81 AA 80 80 80 80 80 80  04 80 F5 F3 E5 F2 A0 A0 A0 
         7D 81 A9 80 80 80 80 80 80  03 80 A0 A0 A0 A0 A0 A0 A0 
         7D 81 A1 80 80 80 80 80 80  
         01 E0 85 B3 96 C1 E1 FF FF 
         01 E0 85 B0 96 C1 E1 FF FF 
         01 E0 85 AE 95 C1 E1 FF FF 
         01 E0 85 AC 95 C1 E1 FF FF 
         01 E0 85 AB 95 C1 E1 FF FF 
         01 E0 85 AA 95 C1 E1 FF FF 
         01 E0 85 AA 95 C1 E1 FF FF 
         01 E0 85 AA 95 C1 E1 FF FF 
         01 E0 85 AA 95 C1 E1 FF FF 
         01 E0 85 A9 95 C1 E1 FF FF 
         01 E0 85 A8 95 C1 E1 FF FF 
         01 E0 85 A7 94 C1 E1 FF FF 
         01 E0 85 A5 94 C1 E1 FF FF 
         01 E0 85 A4 94 C1 E1 FF FF 
         01 E0 85 A2 94 C1 E1 FF FF 
         01 E0 85 A1 94 C1 E1 FF FF 
         01 E0 85 9F 93 C1 E1 FF FF 
         01 E0 85 9E 93 C1 E1 FF FF 
         01 E0 85 9D 93 C1 E1 FF FF 
         01 E0 85 9C 93 C1 E1 FF FF 
         01 E0 85 9B 93 C1 E1 FF FF 
         01 E0 85 9A 93 C1 E1 FF FF 
         01 E0 85 99 93 C1 E1 FF FF 
         01 E0 85 98 93 C1 E1 FF FF  
         01 E0 85 98 93 C1 E1 FF FF 
         01 E0 85 97 92 C1 E1 FF FF 
         01 E0 85 97 92 C1 E1 FF FF 
         01 E0 85 97 92 C1 E1 FF FF 
         01 E0 85 97 92 C1 E1 FF FF 
         01 E0 85 97 92 C1 E1 FF FF 
         01 E0 85 97 92 C1 E1 FF FF 
         01 E0 85 97 92 C1 E1 FF FF 
         01 E0 85 96 92 C1 E1 FF FF 
         01 E0 85 95 92 C1 E1 FF FF 
         01 E0 85 94 92 C1 E1 FF FF 
         01 E0 85 93 92 C1 E1 FF FF 
         01 E0 85 92 92 C1 E1 FF FF 
         01 E0 85 91 92 C1 E1 FF FF 
         01 E0 85 91 92 C1 E1 FF FF 
         01 E0 85 94 92 C1 E1 FF FF  
         01 E0 85 98 93 C1 E1 FF FF 
         01 E0 85 A0 94 C1 E1 FF FF 
         01 E0 85 A9 95 C1 E1 FF FF 
         01 E0 85 B4 96 C1 E1 FF FF 
         01 E0 85 BE 97 C1 E1 FF FF 
         01 E0 85 C7 98 C1 E1 FF FF 
         01 E0 85 CC 99 C1 E1 FF FF 
         01 E0 85 CE 99 C1 E1 FF FF 
         01 E0 85 CD 99 C1 E1 FF  
 FF 01 E0 85 CB 99 C1 E1 FF FF 01 E0 85 C9 99 C1  
 E1 FF FF 01 E0 C5 C7 98 C1 E1 FF FF 01 E0 C5 C5  
 98 C1 E1 FF FF 01 E0 85 C2 98 C1 E1 FF FF 01 E0  
 85 BF 97 C1 E1 FF FF 01 E0 85 BB 97 C1 E1 FF FF  
 01 E0 85 B8 97 C1 E1 FF FF 01 E0 85 B5 96 C1 E1  
 FF FF 01 E0 85 B2 96 C1 E1 FF FF 01 E0 85 B0 96    
 C1 E1 FF FF 01 E0 85 B0 96 C1 E1 FF FF 31 30 35

"""
preamble = "7D 81 A2 80 80 80 80 80 80 \
            7D 81 A7 80 80 80 80 80 80 \
            7D 81 A8 80 80 80 80 80 80 \
            7D 81 A9 80 80 80 80 80 80 \
            7D 81 AA 80 80 80 80 80 80 \
            7D 81 B0 80 80 80 80 80 80"

# starts streaming E1 data (pulse)

cmd1 = "7D 81 A1 80 80 80 80 80 80"

# sent periodically -- to get spo2? E2 data?
cmd1 = "7D 81 A1 80 80 80 80 80 80"
cmd2 = "7d 81 af 80 80 80 80 80 80"

pre2 =     "7D 81 A7 80 80 80 80 80 80  0C 80 \
            7D 81 A2 80 80 80 80 80 80  0C 80 \
            7D 81 A0 80 80 80 80 80 80  06 80 80 87 \
            7D 81 B0 80 80 80 80 80 80  11 80 81 81 80 80 80 80 80 \
            7D 81 AC 80 80 80 80 80 80"

usage = 'Command line usage: \
"P"    -- poll for flowure (ctl-C to exit) \
"sp" f -- set flow point to float f \
"si" i -- set flow point to integer 0 < i < 100'


def chartx(x, clen=20):
    # generate a string which is a bargraph of floating point x
    if x > 1.:
        x = 1.
    slen = int(x*clen)
    cstr = ['%' for s in range(slen)]
    return "".join(cstr)

if __name__ == '__main__':
    
    portname = "/dev/ttyUSB0"
    portname = "COM7"
    portbaud = 115200
    ser = serial.Serial(portname, portbaud, timeout=0.0)
    print('opened port ' + portname + ' at ' + str(portbaud) 
          + ' baud for device')
    sys.stdout.flush()


    cmd1 = [int(s,16) for s in cmd1.split()]
    cmd2 = [int(s,16) for s in cmd2.split()]
    #print(pre_bytes)
    
    ser.write(cmd1)
    ser.write(cmd2)
    time.sleep(0.05)
        #if ser.in_waiting > 0:
        #    inbyte = ser.read(1)
        #    print("got" + str(inbyte))
    count = 0
    while True:
        if ser.in_waiting >= 9:
            count += 1
            inbytes = ser.read(9)
            if inbytes[0] == 0x01 and inbytes[1] == 0xe0:
                print("got E1 " + str(inbytes))
                #streaming pulse data
                print("pulse: {}".format(int(inbytes[5] & 0x7f)))
                print("spO: {}".format(int(inbytes[6] & 0x7f)))
                #print("SpO2: {} ".format(int(inbytes[4])))
                #print(chartx((inbytes[4] - 146)/8.)) 
            elif inbytes[0] == 0x01 and inbytes[1] == 0xe2:
                print("got E2 " + str(inbytes))
                sys.stdout.flush()
            elif inbytes[0] == 0x01 and inbytes[1] == 0xe1:
                # not connected
                print("E2 no data")
               
            else:
                print("got " + str(inbytes))
            time.sleep(0.0001)
            
            if count > 100:
                count = 0
                print("sent cms2")
                ser.write(cmd2)
                sys.stdout.flush()


           
    exit()
