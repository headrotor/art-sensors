## Python files to obtain data from CONTEC CMS50D+ Pulse Oximeter

WORK PRESENTED HERE IS FOR EDUCATIONAL PURPOSES ONLY AND IS UNFIT FOR
ANY HEALTH OR MEDICAL USE INCLUDING BUT NOT LIMITED TO DIAGNOSIS AND
MONITORING.

This Python software obtains streaming and recorded pulse and peripheral blood oxygen saturation (SpO2) data from a CONTEC CMS50D+ Pulse Oximeter

It turns out that the micro-B connector on the device is NOT UBS. It's
a sneaky serial port on a USB connector! This is not a good design,
but it sure is handy for reverse-engineering purposes. The device
instantiates as a USB-serial connector but it does that with
only the cable plugged in: the USB-serial converter is in the cable!
Which meant it was relatively straightforward to reverse-engineer the
undocumented serial protocol to get the data from the device.

First of all, it communicates at 15200 baud with the following pinout:
```
# Micro-B pinout (flat side up, looking INTO device female jack)
#
# _____________
# | 1 2 3 4 5 |   
# \._________./
#  
# Pin 2: device RX (Host TX) (USB white?)
# Pin 3: device TX (Host RX) (USB green?)
# Pin 5: GND                 (USB black)
# other pins: NC?
```

Or you can just use the cable that came with it, noting the serial port (COMn on Windows or /dev/ttyUSBn on Linux) that it comes up as.  

All communication to and from the device seems to occur in 9-byte
packets. The first byte in each packet from the device is `0x01`, all
other bytes have the high bit set, so the low bit in the first byte is
a good delimiter.

The oximeter has two communication modes: a real-time streaming mode,
and a download stored data mode. I've written two Python files to
obtain data from each mode. `stream-data.py` obtains real-time pulse
data and prints a bargraph to the terminal, or you can use it for your
own purposes. `upload_data.py` uploads data recorded in flash and prints it to stdout in csv format.

Command packets from the host are all 9 bytes starting with `0x7D
0x81`. The third byte seems to be some kind of opcode.  Remaining
bytes are padded with 0x80 although there may be data in a command I
have not captured. Again the high bit is set in all bytes except the
first. While there seem to be a number of control packets, the only
two which seem necessary are

`D 81 A1 80 80 80 80 80 80` (to start streaming) and `` (to start downloading realtime data).

The device sends other data back, but data packets are documented in
more detail in the python code, along with sample responses.

