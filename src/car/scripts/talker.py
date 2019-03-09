#!/usr/bin/env python
# license removed for brevity
import rospy
import smbus
import re
import math
import struct
import time
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
IMU_ADD = 0x50
SAVE = 0x00
CALSW = 0X01
RSW = 0X02
RATE = 0x03
BAUD = 0x04
AXOFFSET = 0x05
AYOFFSET = 0x06
AZOFFSET = 0x07
GXOFFSET = 0x08
GYOFFSET = 0x09
GZOFFSET = 0x0a
HXOFFSET = 0x0b
HYOFFSET = 0x0c
HZOFFSET = 0x0d
YYMM = 0x30
DDHH = 0x31
MMSS = 0x32
MS = 0x33
AX = 0x34
AY = 0x35
AZ = 0x36
GX = 0x37
GY = 0x38
GZ = 0x39
HX = 0x3a
HY = 0x3b
HZ = 0x3c
ROLL = 0x3d
PITCH = 0x3e
YAW = 0x3f
bus = smbus.SMBus(1)

def decode(data):
  H = data & 0x00ff
  L = data >> 8
  result = H<<8 | L
  if result>32768:
    result = result - 32768*2
  return result

def gettime():
  yymm = bus.read_word_data(IMU_ADD,YYMM)
  yymm = decode(yymm)
  ddhh = bus.read_word_data(IMU_ADD,DDHH)
  ddhh = decode(ddhh)
  mmss = bus.read_word_data(IMU_ADD,MMSS)
  mmss = decode(mmss)
  ms = bus.read_word_data(IMU_ADD,MS)
  print "20"+str(yymm>>8)+"."+str(yymm&0x00ff)+"."+str(ddhh>>8)
  print str(ddhh&0x00ff)+":"+str(mmss>>8)+":"+str(mmss&0x00ff)+"."+str(((ms&0x00ff)<<8) | (ms>>8))

def getacc(data):
  ax = bus.read_word_data(IMU_ADD,AX)
  ax = decode(ax)
  ay = bus.read_word_data(IMU_ADD,AY)
  ay = decode(ay)
  az = bus.read_word_data(IMU_ADD,AZ)
  az = decode(az)
  ax = float(ax)/32768*16
  ay = float(ay)/32768*16
  az = float(az)/32768*16
  print "acc: x="+str(ax)+" y="+str(ay)+" z="+str(az)
  data.append(ax),data.append(ay),data.append(az)

def getdps(data):
  gx = bus.read_word_data(IMU_ADD,GX)
  gx = decode(gx)
  gy = bus.read_word_data(IMU_ADD,GY)
  gy = decode(gy)
  gz = bus.read_word_data(IMU_ADD,GZ)
  gz = decode(gz)
  gx = float(gx)/32768*2000
  gy = float(gy)/32768*2000
  gz = float(gz)/32768*2000
  print "dps: x="+str(gx)+" y="+str(gy)+" z="+str(gz)
  data.append(gx),data.append(gy),data.append(gz)

def getang():
  roll = bus.read_word_data(IMU_ADD,ROLL)
  roll = decode(roll)
  pitch = bus.read_word_data(IMU_ADD,PITCH)
  pitch = decode(pitch)
  yaw = bus.read_word_data(IMU_ADD,YAW)
  yaw = decode(yaw)
  roll = float(roll)/32768*180
  pitch = float(pitch)/32768*180
  yaw = float(yaw)/32768*180
  print "roll="+str(roll)+" pitch="+str(pitch)+" yaw="+str(yaw)

def talker():
  pub = rospy.Publisher('IMU_data', Imu, queue_size=10)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  bus.write_word_data(IMU_ADD,CALSW,0x01)
  time.sleep(5)
  bus.write_word_data(IMU_ADD,CALSW,0x00)
  while not rospy.is_shutdown():
    acc = []
    dps = []
    imuMsg = Imu()
    imuMsg.header.stamp, imuMsg.header.frame_id = rospy.get_rostime(), "base_link"
    getacc(acc)
    getdps(dps)
    imuMsg.linear_acceleration.x = acc[0]
    imuMsg.linear_acceleration.y = acc[1]
    imuMsg.linear_acceleration.z = acc[2]
    imuMsg.angular_velocity.x = dps[0]
    imuMsg.angular_velocity.y = dps[1]
    imuMsg.angular_velocity.z = dps[2]
    pub.publish(imuMsg)
    rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass