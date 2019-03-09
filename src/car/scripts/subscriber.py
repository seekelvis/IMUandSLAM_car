#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String
ser = serial.Serial("/dev/ttyAMA0", 9600)
count = 0
def callback(data):
    global count
    print count
    count +=1
    if data.data == 'ww':
        ser.write("KR-0,50,0;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == 'ss':
        ser.write("KR-180,50,0;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == 'aa':
        ser.write("KR-270,50,0;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == 'dd':
        ser.write("KR-90,50,0;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == 'qq':
        ser.write("KR-0,0,150;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == 'ee':
        ser.write("KR-0,0,50;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == 'begin':
        ser.write("KR-270,80,50;")
    if data.data == 'stop':
        ser.write("KR-0,0,0;")      
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("car_cmd", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
