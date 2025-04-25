#!/usr/bin/python2

import time
import serial
import struct
import math

packet_hex = "B8 B7 01 D2 12 00 00 00 00 00 52 F7 FF FF 04 00 00 00 00 38 46 00 00"
data = [int(x, 16) for x in packet_hex.split()]
checksum = ((~sum(data) & 0xFF) + 1) & 0xFF
print("Calculated Checksum:", hex(checksum))    