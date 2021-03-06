#!/usr/bin/env python3
import rospy, os, sys

from std_msgs.msg import Header, String, Int32, Float64
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy

vel_msg = Twist()

def callback_controller(data):
    global vel_msg
    
    vel_msg.linear.x = data.axes[2]
    vel_msg.linear.y = data.axes[4]
    vel_msg.angular.z = data.axes[3]

def commander():
    global vel_msg
    vel_msg_head = Twist()



    pub = rospy.Publisher('/four_wheel_steering_controller/cmd_vel', Twist, queue_size = 10)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        pub.publish(vel_msg)

        rate.sleep()

if __name__ == '__main__':

    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("joy0", Joy, callback_controller)
    commander()
    rospy.spin()
