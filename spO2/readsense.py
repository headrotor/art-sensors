#!/usr/bin/python
# -*- coding: utf-8 -*-
# hardware interface to Alicat flow controllser


import serial
import sys
import time
import threading
import traceback

#local serial wrapper
import serial_io

class HWPoller(object):
    """ thread to repeatedly poll hardware
    sleeptime: time to sleep between pollfunc calls
    pollfunc: function to repeatedly call to poll hardware"""
  
    def __init__(self, hwname, pollfunc, pollrate=0.02):
        self.pollfunc = pollfunc  
        self.hwname = hwname
        self.pollrate = pollrate
        self.thread = threading.Thread(target=self.worker)
        #self.thread = TracebackLoggingThread(target=self.worker)
        self._run_flag = threading.Event()
        self._run_flag.set()
        self.thread.daemon = True
        self.thread.start()

    def worker(self):
        while(self._run_flag):
            if True:
                sys.stdout.flush()
                try:
                    self.pollfunc()
                except Exception: # catch exception, 
                    traceback.print_exc()
                time.sleep(self.pollrate)
            else:
                time.sleep(self.pollrate)


    def stop(self):
        self._run_flag.clear()

    def kill(self):
        self.stop()
        sys.stdout.flush()
        #self.thread.join()

class alicat(object):

    def __init__(self, ser, cfg):
        self.cfg = cfg
        self.ser = ser
        self.max_range_PSI = 5.0
        self.verbose = False

        self.sleeptime = self.cfg.getfloat('alicat','sleeptime')
        self.flow = -1.0  # current flowure from thread
        self.setpt = -1.0  # current setpoint from thread
        self.interlock = False  # set to prohibit thread from accessing serial port
        if self.ser.stdout is True:
            return
        self.Poller = HWPoller('alicat', 
                               self.query_flowure_interlock,
                               self.sleeptime)
        #self.worker = GetFlowureWorker(self.sleeptime,
        #        self.query_flowure_interlock)
        #self.worker.resume()
        #self.worker.start()


    def init_alicat(self):
        """Initialize flowure controller from config file,
        return False if error"""

        self.verbose = False

      # OK, ready to start flowure thread

        #print "started!"
        self.interlock = False
        sys.stdout.flush()
        return True



    def pollfunc(self):
        #print "Hi!"
        sys.stdout.flush()


    def set_setpoint_int(self, setp_i):
        # setpoint value is (setp_i/64000) * fullscale
        cmd_str = 'A%d\r\n' % int(setp)
        self.send_cmd(cmd_str)
        
        
    def set_setpoint_float(self, setp_f):
        cmd_str = 'AS%05.2f\r\n' % float(setp_f)
        self.send_cmd(cmd_str)

    def send_cmd(self, cmd_str):
        """Set the flowure setpoint in PSI"""

        if self.ser.stdout:
            return
        self.interlock = True
        self.ser.ser.write(cmd_str)
        self.ser.ser.flush()
        time.sleep(0.1)
        self.interlock = False

    def TestConnected(self):
        """ Return FALSE if there is an issue with the serial connection"""

        if self.ser.stdout:
            return True
        if self.ser.ser == None:
            return False
        self.interlock = True
        self.flow = 'test'
        time.sleep(0.3)
        self.query_flowure()
        if self.flow == 'test':
            return False
        return True


    def get_flow_mmHG(self):
        return self.flow * 51.7

    def get_flow(self):
        """ return the last flowure reading, as a float,
        -1 if there was an err"""
        return self.flow

    def query_flowure(self):
        """queries, reads and parses flowure ctrl"""

        if self.ser.stdout:
            return
        self.ser.checkwrite('A\r')
        time.sleep(0.005)
        response = self.ser.readline_n(34, tsleep=0.0)
        #response = self.ser.checkread(33)
        #print len(response)
        

        response = response.strip()
        fields = response.split()

    def set_p_reg(self, p_val):
        """Set the proportional PID term on the Alicat controller"""
        return self.set_register(21, int(p_val))

    def set_d_reg(self, d_val):
        """Set the derivative PID term on the Alicat controller"""
        return self.set_register(22, int(d_val))

    def set_register(self, reg, val):
        """Set register 'reg' to value 'val' and confirm response"""
        if self.ser.stdout:
            return True

        self.interlock = True
        time.sleep(2 * self.sleeptime)
        set_str = '*W%d=%d\r' % (int(reg), int(val))
        self.ser.checkwrite(set_str)


        # construct estimated response string (to get length)
        # reponse string is of form "A   021 = 1000"



        response_str = self.ser.readline_n(20)

        # check response is what we wrote
        #est_response = 'A   %03d = %d\r' % (int(reg), int(val))
        response = response_str.split()

        if len(response) < 4:
            print("alicat: error parsing register set response")
            return False
        
        #print 'got response: ' + repr(response_str)

        return True


preamble = "7D 81 A2 80 80 80 80 80 80 \
            7D 81 A7 80 80 80 80 80 80 \
            7D 81 A8 80 80 80 80 80 80 \
            7D 81 A9 80 80 80 80 80 80 \
            7D 81 AA 80 80 80 80 80 80 \
            7D 81 B0 80 80 80 80 80 80"

pre2 =     "7D 81 A7 80 80 80 80 80 80  0C 80 \
            7D 81 A2 80 80 80 80 80 80  0C 80 \
            7D 81 A0 80 80 80 80 80 80  06 80 80 87 \
            7D 81 B0 80 80 80 80 80 80  11 80 81 81 80 80 80 80 80 \
            7D 81 AC 80 80 80 80 80 80"

usage = 'Command line usage: \
"P"    -- poll for flowure (ctl-C to exit) \
"sp" f -- set flow point to float f \
"si" i -- set flow point to integer 0 < i < 100'

if __name__ == '__main__':
    
    portname = "/dev/ttyUSB0"
    portbaud = 115200
    ser = serial_io.SerialIO(portname, portbaud, timeout=0.0)
    print('opened port ' + portname + ' at ' + str(portbaud) 
          + ' baud for device')
    sys.stdout.flush()


    pre_bytes = [int(s,16) for s in pre2.split()]
    print(pre_bytes)
    
    sendstr = bytes(pre_bytes)
    
    for b in sendstr:
        
        ser.ser.write(pre_bytes[0:8])
        time.sleep(0.05)
        if ser.ser.in_waiting > 0:
            inbyte = ser.ser.read(1)
            print("got" + str(inbyte))



    time.sleep(0.5)
    while True:
        if ser.ser.in_waiting > 0:
            inbyte = ser.ser.read(1)
            print("got %x" % int(inbyte))
        time.sleep(0.001)

           
    exit()
