#!/usr/bin/python
# -*- coding: utf-8 -*-
# reverse-engineered interface to upload data recorded from
# CONTEC Pulse Oximeter Model model CMS50D+

# Note USB-serial converter is in the CABLE: micro-B connector on device is serial, NOT USB.

### WARNING: UNFIT FOR ANY MEDICAL USE INCLUDING BUT NOT LIMITED TO DIAGNOSIS AND MONITORING
### NO WARRANTY EXPRESS OR IMPLIED -- UNDOCUMENTED SERIAL INTERFACE: USE AT OWN RISK

# communication is over the USB-serial converter at 115200 baud, 8 bits, no parity or handshake.
# Serial protocol seems to be mostly 9-byte packets.

# Commands are all 9 bytes starting with  0x7D 0x81. 
# The third byte seems to be some kind of opcode. 
# Remaining bytes are padded with 0x80 although there may be data in a command I have not captured. 
# In the response, the high bit is set on all but the first byte. 
# Most responses are 

import serial
import sys
import time

# None to not log
logfn = "contec.log"


"""
    Captured data from serial port: on each line first 9 bytes are sent, remaining bytes are response
    Left column is sent data (9-byte packets), right data is response from device
    7d 81 a7 80 80 80 80 80 80               0c 80       
    7d 81 a2 80 80 80 80 80 80               0c 80                                          
    7d 81 a0 80 80 80 80 80 80               06 80 80 87                                    
    7d 81 b0 80 80 80 80 80 80               11 80 81 81 80 80 80 80 80                     
    7d 81 ac 80 80 80 80 80 80               0e 80 81                   
    7d 81 b3 80 80 80 80 80 80               13 80 a0 a0 a0 a0 a0 a0 a0 
                                             14 80 a0 a0 a0 a0 a0 80 80                  
    7d 81 ad 80 80 80 80 80 80               10 80 81                                      
    7d 81 a3 80 80 80 80 80 80               0a 80 80 81                                   
    7d 81 ab 80 80 80 80 80 80               05 80 80 f5 f3 e5 f2 80 80                   
    7d 81 a4 80 80 80 80 80 80               08 80 80 80 be 80 80 80                      
    7d 81 a5 80 80 80 80 80 80               07 80 80 80 80 80 80 80 
                                             12 80 80 80 80 80 80 80  
    7d 9f 81 be 80 80 80 80 80               0b 81 81 85                                   
    7d 81 af 80 80 80 80 80 80           
    7d 81 af 80 80 80 80 80 80           
    7d 81 af 80 80 80 80 80 80           
    7d 81 af 80 80 80 80 80 80           
    7d 81 af 80 80 80 80 80 80           
    7d 81 a7 80 80 80 80 80 80               0c 80                                         
    7d 81 a2 80 80 80 80 80 80               0c 80        
    7d 81 a6 80 80 80 80 80 80               0f 80 80 80 80 80 80 80 # start of data? 11 packets 
                                             0f 80 80 80 80 80 80 80 # length = 0x0B, 0.5 hz? 
                                             0f 80 80 80 80 80 80 80 # expect to see 0x8b above
                                             0f 80 80 80 80 80 80 80 # 6 bytes per packet, 
                                             0f 80 80 80 de be de be 
                                             0f 80 df bd df bc df b9
                                             0f 80 e1 b7 e2 bb e2 bb 
                                             0f 80 e2 bb e2 bb e2 bf
                                             0f 80 e2 be e2 be e2 be 
                                             0f 80 e2 be e2 be e2 be
                                             0f 80 e2 be 80 80 80 80               
    7d 81 a7 80 80 80 80 80 80               0c 80                                            
    7d 81 af 80 80 80 80 80 80           
    7d 81 a7 80 80 80 80 80 80
    7d 81 a2 80 80 80 80 80 80               0c 80 0c 80                                    





         7D 81 A7 80 80 80 80 80 80  0C 80 
         7D 81 A2 80 80 80 80 80 80  0C 80 
         7D 81 A0 80 80 80 80 80 80  06 80 80 87 
         7D 81 B0 80 80 80 80 80 80  11 80 81 81 80 80 80 80 80 
         7D 81 AC 80 80 80 80 80 80  0E 80 81 
         7D 81 B3 80 80 80 80 80 80  13 80 A0 A0 A0 A0 A0 A0 A0 14 80 A0 A0 A0 A0 A0 80 80  
         7D 81 A8 80 80 80 80 80 80  02 80 80 A0 A0 A0 A0 A0 A0 02 81 FF A0 A0 A0 A0 A0 A0 
         7D 81 AA 80 80 80 80 80 80  04 80 F5 F3 E5 F2 A0 A0 A0 
         7D 81 A9 80 80 80 80 80 80  03 80 A0 A0 A0 A0 A0 A0 A0 
         7D 81 A1 80 80 80 80 80 80  01 E0 85 B3 96 C1 E1 FF FF 
                                     01 E0 85 B0 96 C1 E1 FF FF 
                                     01 E0 85 AE 95 C1 E1 FF FF 
                                     01 E0 85 AC 95 C1 E1 FF FF 
                                     01 E0 85 AB 95 C1 E1 FF FF 
                                     01 E0 85 AA 95 C1 E1 FF FF 
                                     01 E0 85 AA 95 C1 E1 FF FF 

         This is streaming data, first byte is 0x01, second is 0xEn where n indicates data validity: 
                                 0xE0 valid real-time and pulse data (takes a few seconds, not shown in trace above)
                                 0xE1 real-time  data is valid, pulse and spo2 not valid
                                 0xE2 no valid data 
                                 next two bytes are real-time pulse data (for graph) (with high bit set?)
                                 (unsure why two different values, but may echo bar graph and plot on device OLED.
                                 next two bytes are pulse in BPM, and spO2 percent (high bit is set so AND with 0x7F to get integer values)
                                 Last two bytes are 0xFF
"""


# starts streaming real-time data (pulse)
cmd1 = "7D 81 A1 80 80 80 80 80 80"

# starts downloading stored data
cmd1 = "7d 81 a6 80 80 80 80 80 80"

# sent periodically by host --  Keep-alive? May not need this. 
cmd2 = "7d 81 af 80 80 80 80 80 80"

def chartx(x, clen=20, c='#'):
    """generate a string of max length clen which is a bargraph of 
    floating point value 0. <= x <= 1 consisting of character c """
    if x > 1.:
        x = 1.
    slen = int(x*clen)
    cstr = [c for s in range(slen)]
    return "".join(cstr)

if __name__ == '__main__':



    
    portname = "/dev/ttyUSB0"
    portbaud = 115200
    ser = serial.Serial(portname, portbaud, timeout=0.0)
    print('opened port ' + portname + ' at ' + str(portbaud) 
          + ' baud for device')
    sys.stdout.flush()

    if logfn is not None:
        logfile = open(logfn, 'w')

    # convert ascii hex to bytes
    cmd1 = [int(s,16) for s in cmd1.split()]
    cmd2 = [int(s,16) for s in cmd2.split()]
    #print(pre_bytes)
    
    ser.write(cmd1)
    #ser.write(cmd2)
    time.sleep(0.05)
        #if ser.in_waiting > 0:
        #    inbyte = ser.read(1)
        #    print("got" + str(inbyte))
    count = 0
    while True:
        if ser.in_waiting >= 8:
            count += 1
            inbytes = ser.read(8)
            print("got " + str(inbytes))
            if inbytes[0] == 0x0f: 
                print("{}, {}".format(int(inbytes[2] & 0x7f), int(inbytes[3] & 0x7f)))
                print("{}, {}".format(int(inbytes[4] & 0x7f), int(inbytes[5] & 0x7f)))
                print("{}, {}".format(int(inbytes[6] & 0x7f), int(inbytes[7] & 0x7f)))

                #logfile.write("{},".format(time.time()))
                #logfile.write("{},{},{},{},{}\n".format(*[inbytes[n] & 0x7F for n in [2,3,4,5,6]]))
            else:
                print("got " + str(inbytes))
            time.sleep(0.0001)
            
            if count > 100:
                count = 0
                print("sent cms2")
                ser.write(cmd2)
                sys.stdout.flush()


           
    exit()
