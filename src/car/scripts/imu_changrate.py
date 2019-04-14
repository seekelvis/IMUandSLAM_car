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
ser = serial.Serial("/dev/ttyAMA0", 460800)
#begin set
ser.write(binascii.a2b_hex('FFAA030900'))
print "begin to change feedback rate to 100hz"
#ser.write(binascii.a2b_hex('FFAA030B00'))
#print "begin to change feedback rate to 200hz"


#ser.write(binascii.a2b_hex('FFAA040700'))
#print "begin to change BUAD rate to 230400hz"
#ser.write(binascii.a2b_hex('FFAA040800'))
#print "begin to change BUAD rate to 460800hz"

