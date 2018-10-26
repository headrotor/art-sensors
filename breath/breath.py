from __future__ import print_function # WalabotAPI works on both Python 2 an 3.
import sys
from sys import platform
from os import system
from imp import load_source
import time
import os
import serial

if platform == 'win32':
    modulePath = os.path.join('C:/', 'Program Files', 'Walabot', 'WalabotSDK', 'Python', 'WalabotAPI.py')
elif platform.startswith('linux'):
    modulePath = os.path.join('/usr', 'share', 'walabot', 'python', 'WalabotAPI.py')


# these are stepper steps so depends on driver settings
# zero should be limit switch



# proportional to number of steps per rotation, kind of arbitrary
step_factor = 300

motor_low = 500
motor_range = 1600



# arbitrary numbers here, between 1 and 20?
gain = 10.

# first order highpass coefficient to remove dc offset
hipass = 0.985
envelope_hipass = 0.992

ser = serial.Serial('COM3', 115200, timeout=0)

wlbt = load_source('WalabotAPI', modulePath)

wlbt.Init()


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

# def display_breath(energy):
#     pos = (energy + 200)/400.
#     if pos > 1.0:
#         pos = 1.0
#     if pos < 0.:
#          pos = 0. 
#     print(chartx(pos))
#     sys.stdout.flush()
    
def BreathingApp():

    bpos = 0.

    # Walabot_SetArenaR - input parameters
    minInCm, maxInCm, resInCm = 30, 150, 1
    # Walabot_SetArenaTheta - input parameters
    minIndegrees, maxIndegrees, resIndegrees = -4, 4, 2
    # Walabot_SetArenaPhi - input parameters
    minPhiInDegrees, maxPhiInDegrees, resPhiInDegrees = -4, 4, 2
    # Configure Walabot database install location (for windows)
    wlbt.SetSettingsFolder()
    # 1) Connect : Establish communication with walabot.
    try:
        wlbt.ConnectAny()
    except wlbt.WalabotError:
        print("walabot not found")
        exit(0)
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

    # floating point position
    fpos = 0.
    max_fpos = -10.
    min_fpos = 10.
    while True:
        appStatus, calibrationProcess = wlbt.GetStatus()
        # 5) Trigger: Scan(sense) according to profile and record signals
        # to be available for processing and retrieval.
        try:
            wlbt.Trigger()
        except wlbt.WalabotError:
            # exit cleanly; wait for restart
            print("disconnected");
            exit(0)
        # 6) Get action: retrieve the last completed triggered recording
        energy = wlbt.GetImageEnergy()
        #energy seems to be some kind of differential signal.
        # positive on inhale (motion away from sensor), negative on exhale.
        #pos = energy_to_steps(energy)
        # integrate energy to get position
        fpos += float(energy) * 10e3 * gain
        # high pass to remove DC and return to zero if no signal
        fpos = fpos * hipass
        max_fpos = max_fpos * envelope_hipass
        min_fpos = min_fpos * envelope_hipass
        # add offset and scale to get stepper step position
        #pos = int(fpos * step_factor) + offset

        # clamp to min and max limits
        #if pos > high_limit:
        #    pos = high_limit
        #if pos < low_limit:
        #   pos = low_limit;

        if fpos > max_fpos:
            max_fpos = fpos
        if fpos < min_fpos:
            min_fpos = fpos


        # do our best to convert to 0-1 range
        pos_range = max_fpos - min_fpos

        # don't divide by zero
        if pos_range < 5:
            pos_range = 5

        #pos_range = 15    
        zpos = (fpos - min_fpos)/pos_range
            
        pos = int(motor_range*zpos) + motor_low
        # send the position data to the stepper controller over the serial port
        ser.write("{:d}\n".format(int(pos)).encode('utf-8'))
        ser.flushOutput()


        # display a bargraph
        
        
        print("{0:6.2f} ".format(zpos))
        print("{0:6.2f} ".format(pos) + chartx(zpos))
        sys.stdout.flush()

        # don't send data faster than the thing can handle
        #time.sleep(0.02)
        #print('{}'.format(fpos))
        #PrintBreathingEnergy(energy)
    # 7) Stop and Disconnect.
    wlbt.Stop()
    wlbt.Disconnect()
    print('Terminate successfully')

if __name__ == '__main__':
    BreathingApp()
