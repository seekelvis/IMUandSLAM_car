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
count = 0
imuMsg = Imu()
def decode(datah,datal):
    result = datah*256+datal
    if result>32768:
        result = result - 32768*2
    return float(result)

def talker():
    global count
    global imuMsg
    pub = rospy.Publisher('IMU_data', Imu, queue_size=10)
    origintime = 0
    realtime = 0
    rospy.init_node('IMU_Publisher', anonymous=True)
    rate = rospy.Rate(2200) # 100hz
    while not rospy.is_shutdown():
        #print "=================================="+str(count)
	#imuMsg = Imu()
        imuMsg.header.stamp, imuMsg.header.frame_id = rospy.get_rostime(), "base_link"
	if origintime == 0:
	    origintime = imuMsg.header.stamp
	realtime = float(imuMsg.header.stamp.secs - origintime.secs) + float(imuMsg.header.stamp.nsecs - origintime.nsecs)/1000000000
	#print "timestamp--"+str(imuMsg.header.stamp)
	data = binascii.b2a_hex(ser.read())
        if data == '55':
            data = binascii.b2a_hex(ser.read())
            if data == '50':
                year = int(binascii.b2a_hex(ser.read()),16)
                month = int(binascii.b2a_hex(ser.read()),16)
                day = int(binascii.b2a_hex(ser.read()),16)
                hour = int(binascii.b2a_hex(ser.read()),16)
                minutes = int(binascii.b2a_hex(ser.read()),16)
                second = int(binascii.b2a_hex(ser.read()),16)
                msl = int(binascii.b2a_hex(ser.read()),16)
                msh = int(binascii.b2a_hex(ser.read()),16)
                #print "20"+str(year)+"."+str(month)+"."+str(day)
                #print str(hour)+":"+str(minutes)+":"+str(second)+"."+str(msh*256+msl)
            if data == '51':
                axl = int(binascii.b2a_hex(ser.read()),16)
                axh = int(binascii.b2a_hex(ser.read()),16)
                ayl = int(binascii.b2a_hex(ser.read()),16)
                ayh = int(binascii.b2a_hex(ser.read()),16)
                azl = int(binascii.b2a_hex(ser.read()),16)
                azh = int(binascii.b2a_hex(ser.read()),16)
                tl = int(binascii.b2a_hex(ser.read()),16)
                th = int(binascii.b2a_hex(ser.read()),16)
                ax = decode(axh,axl)/32768*16
                ay = decode(ayh,ayl)/32768*16
                az = decode(azh,azl)/32768*16
                t = float(th*256+tl)/100
                imuMsg.linear_acceleration.x = ax
                imuMsg.linear_acceleration.y = ay
                imuMsg.linear_acceleration.z = az
         #       print "acc: ("+str(ax)+", "+str(ay)+", "+str(az)+")"
            if data == '52':
                wxl = int(binascii.b2a_hex(ser.read()),16)
                wxh = int(binascii.b2a_hex(ser.read()),16)
                wyl = int(binascii.b2a_hex(ser.read()),16)
                wyh = int(binascii.b2a_hex(ser.read()),16)
                wzl = int(binascii.b2a_hex(ser.read()),16)
                wzh = int(binascii.b2a_hex(ser.read()),16)
                tl = int(binascii.b2a_hex(ser.read()),16)
                th = int(binascii.b2a_hex(ser.read()),16)
                wx = decode(wxh,wxl)/32768*2000
                wy = decode(wyh,wyl)/32768*2000
                wz = decode(wzh,wzl)/32768*2000
                imuMsg.angular_velocity.x = wx
                imuMsg.angular_velocity.y = wy
                imuMsg.angular_velocity.z = wz
                t = float(th*256+tl)/100
	#	print "gyr:(roll,pitch,yaw)  ("+str(wx)+", "+str(wy)+", "+str(wz)+")"
            if data == '53':
                rolll = int(binascii.b2a_hex(ser.read()),16)
                rollh = int(binascii.b2a_hex(ser.read()),16)
                pitchl = int(binascii.b2a_hex(ser.read()),16)
                pitchh = int(binascii.b2a_hex(ser.read()),16)
                yawl = int(binascii.b2a_hex(ser.read()),16)
                yawh = int(binascii.b2a_hex(ser.read()),16)
                tl = int(binascii.b2a_hex(ser.read()),16)
                th = int(binascii.b2a_hex(ser.read()),16)
                roll = decode(rollh,rolll)/32768*180
                pitch = decode(pitchh,pitchl)/32768*180
                yaw = decode(yawh,yawl)/32768*180
                t = float(th*256+tl)/100
	#	print "ang:(roll,pitch,yaw) ("+str(roll)+", "+str(pitch)+", "+str(yaw)+")"
	    	pub.publish(imuMsg)
		count = count + 1
		print "=================================="+str(count),"============",realtime
		#print "timestamp--"+str(imuMsg.header.stamp)
		print "acc: ("+str(imuMsg.linear_acceleration.x)+", "+str(imuMsg.linear_acceleration.y)+", "+str(imuMsg.linear_acceleration.z)+")"
                #print "gyr:(roll,pitch,yaw)  ("+str(imuMsg.angular_velocity.x)+", "+str(imuMsg.angular_velocity.y)+", "+str(imuMsg.angular_velocity.z)+")"
                #print "ang:(roll,pitch,yaw) ("+str(roll)+", "+str(pitch)+", "+str(yaw)+")"

            	rate.sleep()
        
        
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
