#!/usr/bin/env python
import rospy
import serial
import serial.tools.list_ports
import binascii
import time
from std_msgs.msg import String

ser_acc = serial.Serial("/dev/ttyAMA0", 115200)

plist = list(serial.tools.list_ports.comports())
if len(plist) <= 0:
	print("Can't find serials")
	ser = serial.Serial("/dev/ttyUSB0", 9600)
else:
	plist_0 = list(plist[0])
	serialName = plist_0[0]
	serialFd = serial.Serial(serialName, 9600, timeout=60)
	print("Available serial >>> ", serialFd.name)
	ser = serial.Serial(serialFd.name, 9600)

count = 0

def callback(data):
    global count
    print count
    count +=1
    
    if data.data == 'init_acc':
	ser_acc.write(binascii.a2b_hex('FFAA010100'))
	print "begin to acc_init"
	time.sleep(10)
	ser_acc.write(binascii.a2b_hex('FFAA010000'))
	print "finish acc_init"

    if data.data == '11':
        ser.write("KR-0,100,0;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == '22':
        ser.write("KR-180,100,0;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == '33':
        ser.write("KR-270,100,0;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == '44':
        ser.write("KR-90,100,0;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")


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
        ser.write("KR-0,0,130;")
        time.sleep(0.1)
        ser.write("KR-0,0,0;")
    if data.data == 'ee':
        ser.write("KR-0,0,30;")
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
