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



ser = serial.Serial('COM6', 115200, timeout=0)

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

def display_breath(energy):
    pos = (energy + 200)/400.
    if pos > 1.0:
        pos = 1.0
    if pos < 0.:
         pos = 0. 
    print(chartx(pos))
    sys.stdout.flush()
    
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
    wlbt.ConnectAny()
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
    while True:
        appStatus, calibrationProcess = wlbt.GetStatus()
        # 5) Trigger: Scan(sense) according to profile and record signals
        # to be available for processing and retrieval.
        wlbt.Trigger()
        # 6) Get action: retrieve the last completed triggered recording
        energy = wlbt.GetImageEnergy()
        # PrintBreathingEnergy(energy)
        #display_breath(10e7 * energy)
        bpos += energy * 10e3 * 8.
        bpos = bpos * 0.993
        pos = int(bpos * 300)
        if pos > 1000:
            pos = 1000
        if pos < -1000:
           pos = -1000;
        ser.write(str(pos))
        ser.write('\n')
        print(chartx((pos + 1000)/2000.))
        print(pos)
        #print('{}'.format(bpos))
        #PrintBreathingEnergy(energy)
    # 7) Stop and Disconnect.
    wlbt.Stop()
    wlbt.Disconnect()
    print('Terminate successfully')

if __name__ == '__main__':
    BreathingApp()
