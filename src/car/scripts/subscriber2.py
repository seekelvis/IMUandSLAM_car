#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

def callback(data):
    rospy.loginfo("acc:  x=%s y=%s z=%s", str(data.linear_acceleration.x),str(data.linear_acceleration.y),str(data.linear_acceleration.z))
    rospy.loginfo("dps:  x=%s y=%s z=%s", str(data.angular_velocity.x),str(data.angular_velocity.y),str(data.angular_velocity.z))

#    for i in range (0,3):
#        v1[i] = v0[i] + data.a[i] * data.dt
#        s1[i] = s0[i] + (v0[i] + v1[i]) * data.dt / 2
#        v0[i] = v1[i]
#        s0[i] = s1[i]
    
def listener():

   
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("IMU_data", Imu, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()