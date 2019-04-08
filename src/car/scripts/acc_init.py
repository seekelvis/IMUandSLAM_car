#!/usr/bin/env python
# license removed for brevity
import rospy
import smbus
import re
import math
import struct
import time
import binascii
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import serial
ser = serial.Serial("/dev/ttyAMA0", 115200)
#begin set
ser.write(binascii.a2b_hex('FFAA010100'))
print "begin to acc_init"
time.sleep(2)
#quit
ser.write(binascii.a2b_hex('FFAA010000'))
print "finish acc_init"
