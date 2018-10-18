from __future__ import print_function # WalabotAPI works on both Python 2 an 3.
import sys
from sys import platform
from os import system
from imp import load_source
import time
import os
import serial

from blink1.blink1 import Blink1

b1 = Blink1()
b1.fade_to_rgb(1000, 64, 64, 64)




if platform == 'win32':
    modulePath = os.path.join('C:/', 'Program Files', 'Walabot', 'WalabotSDK', 'Python', 'WalabotAPI.py')
elif platform.startswith('linux'):
    modulePath = os.path.join('/usr', 'share', 'walabot', 'python', 'WalabotAPI.py')


# these are stepper steps so depends on driver settings
# zero should be limit switch

low_limit = 0
high_limit = 1600
# kind of the center point, return here at zero signal
offset = high_limit/3

# arbitrary numbers here, between 1 and 20?
gain = 15.

# first order highpass coefficient to remove dc offset
hipass = 0.985

try:
    ser = serial.Serial('COM3', 115200, timeout=0)
except serial.SerialException:
    print("could not find teensy, continuing")
    ser = None
    b1.fade_to_rgb(10, 0, 255 ,255) # yellow means no teensy

    
wlbt = load_source('WalabotAPI', modulePath)

wlbt.Init()


# def energy_to_steps(e):
#     bpos += energy * 10e3 * 8.
#     bpos = bpos * 0.995
#     pos = int(bpos * 300)
#     if pos > 1000:
#         pos = 1000
#     if pos < -1000:
#         pos = -1000;
    
def chartx(x, clen=70, c='#'):
    """generate a string of max length clen which is a bargraph of 
    floating point value 0. <= x <= 1 consisting of character c """
    if x > 1.:
        x = 1.
    slen = int(x*clen)
    cstr = [c for s in range(slen)]
    return "".join(cstr)


def PrintBreathingEnergy(energy):
    #system('cls' if platform == 'win32' else 'clear')
    #print('{0}, {1}'.format(time.time(),int(10e7*energy)))
    print('{0:4d}'.format(int(10e7*energy)))
    sys.stdout.flush()

def display_breath(energy):
    pos = (energy + 200)/400.
    if pos > 1.0:
        pos = 1.0
    if pos < 0.:
         pos = 0. 
    print(chartx(pos))
    sys.stdout.flush()


the_state = 'xrest'
# time of last transition
last_transition = time.time()
low_pass = 0.0

def handle_state(delta_e):
    global the_state
    global last_transition
    global low_pass
    ''' detect inhale and exhales by delta energy'''

    a = 0.97
    low_pass = a*low_pass + (1-a)*delta_e 
    min_i_thresh = 100 # need positive delta above this
    max_i_thresh = 1200 # need positive delta below this (above this is motion)

    min_x_thresh = -100
    max_x_thresh = -1200

    delta_e = low_pass

    #print("{:05.1f}".format(0.1*delta_e))
    display_breath(delta_e*0.2)
    
    if the_state == 'xrest':
        if delta_e > min_i_thresh and delta_e < max_i_thresh:
            the_state = 'inhale'
            print(the_state)
            last_transition = time.time()
            print("inhale at" + str(delta_e))
            if ser is not None:
                ser.write('i\n'.encode('utf-8'))
            
    elif the_state == 'inhale':
        if time.time() - last_transition  > 1.0:
            the_state = 'irest'
            print(the_state)
            last_transition = time.time()

    elif the_state == 'irest':
        if delta_e < min_x_thresh and delta_e > max_x_thresh:
            print("exhale at" + str(delta_e))
            the_state = 'exhale'
            print(the_state)
            last_transition = time.time()
            if ser is not None:
                ser.write('x\n'.encode('utf-8'))

    elif the_state == 'exhale':
        if time.time() - last_transition  > 1.0:
            the_state = 'xrest'
            print(the_state)
            last_transition = time.time()

                
def BreathingApp():

    bpos = 0.
    detect_thresh = 1.0

    
    # Walabot_SetArenaR - input parameters
    minInCm, maxInCm, resInCm = 30, 150, 1
    # Walabot_SetArenaTheta - input parameters
    minIndegrees, maxIndegrees, resIndegrees = -4, 4, 2
    # Walabot_SetArenaPhi - input parameters
    minPhiInDegrees, maxPhiInDegrees, resPhiInDegrees = -4, 4, 2
    # Configure Walabot database install location (for windows)
    wlbt.SetSettingsFolder()
    # 1) Connect : Establish communication with walabot.

    while True:
        try:
            wlbt.ConnectAny()
        except wlbt.WalabotError:
            print("could not connect to walabot") 
            b1.fade_to_rgb(10, 255, 0 ,0)
            time.sleep(1.0)
            sys.stdout.flush()
        else:
            if ser is not None: # we opened serial port OK, go green
                b1.fade_to_rgb(10, 0, 255 ,0)
            else:
                b1.fade_to_rgb(10, 0, 255 ,255) # yellow means no teensy
            break # walabot OK, break out of loop

        
    # 2) Configure: Set scan profile and arena
    # Set Profile - to Sensor-Narrow.
    wlbt.SetProfile(wlbt.PROF_SENSOR_NARROW)
    # Setup arena - specify it by Cartesian coordinates.
    wlbt.SetArenaR(minInCm, maxInCm, resInCm)
    # Sets polar range and resolution of arena (parameters in degrees).
    wlbt.SetArenaTheta(minIndegrees, maxIndegrees, resIndegrees)
    # Sets azimuth range and resolution of arena.(parameters in degrees).
    wlbt.SetArenaPhi(minPhiInDegrees, maxPhiInDegrees, resPhiInDegrees)
    # Dynamic-imaging filter for the specific frequencies typical of breathing
    wlbt.SetDynamicImageFilter(wlbt.FILTER_TYPE_DERIVATIVE)
    # 3) Start: Start the system in preparation for scanning.
    wlbt.Start()
    # 4) Trigger: Scan (sense) according to profile and record signals to be
    # available for processing and retrieval.
    bpos = 0.
    e = 0;
    laste = 0;
    print("waiting to inhale")
    if ser is not None: # exhale so we staert in the exhaled state
        ser.write('x\n'.encode('utf-8'))
    
    while True:
        appStatus, calibrationProcess = wlbt.GetStatus()
        # 5) Trigger: Scan(sense) according to profile and record signals
        # to be available for processing and retrieval.
        wlbt.Trigger()
        # 6) Get action: retrieve the last completed triggered recording
        energy = wlbt.GetImageEnergy()
        laste = e;
        e = float(energy) * 10e3 * gain
        #energy seems to be some kind of differential signal.
        # positive on inhale (motion away from sensor), negative on exhale.
        #pos = energy_to_steps(energy)
        # integrate energy to get position
        bpos += float(energy) * 10e3 * gain
        # high pass to remove DC and return to zero if no signal
        bpos = bpos * hipass
        # add offset and scale to get stepper step position
        pos = int(bpos * 300.) + offset
        # clamp to min and max limits
        if pos > high_limit:
            pos = high_limit
        if pos < low_limit:
           pos = low_limit;

        # send the position data to the stepper controller over the serial port
        #ser.write(str(pos))
        #ser.write('\n')

        # display a bargraph
        deltae = 10e3* (e - laste)
        handle_state(deltae)
        
        #print("{0:6.4f} ".format(10e3*laste))
        #print("{0:6.4f} ".format(deltae))
        #print("{0:6.2f} ".format(bpos) + chartx(float(pos - low_limit)/float(high_limit - low_limit)))
        sys.stdout.flush()

        # don't send data faster than the thing can handle
        time.sleep(0.001)
        #print('{}'.format(bpos))
        #PrintBreathingEnergy(energy)
    # 7) Stop and Disconnect.
    wlbt.Stop()
    wlbt.Disconnect()
    print('Terminate successfully')

if __name__ == '__main__':
    BreathingApp()
