#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Int64

def talker():
    pub = rospy.Publisher('battery_server', Int64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = 90
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rospy.sleep(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass