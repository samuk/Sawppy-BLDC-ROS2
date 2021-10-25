#! /usr/bin/env python

import rospy
import tf
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

#Variables
M_PI = 3.14159265358979323846

#subscriber callbacks
def cb_imu(msg):
    quaternion = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2] * 180.0 / M_PI - 90 # to degrees amd account for 90 deg orientation offset
    if yaw < 0:
        yaw += 360
    pub.publish(yaw)

if __name__ == '__main__':
    rospy.init_node('imu_transform', anonymous=True)

    #Publishers and subscribers
    pub = rospy.Publisher('/imu_deg', Float32, queue_size=1) # output imu heading in deg from [0,360) with corrected 90deg offset
    rospy.Subscriber('/imu_in', Imu, cb_imu)    #input imu topic

    rospy.spin()
