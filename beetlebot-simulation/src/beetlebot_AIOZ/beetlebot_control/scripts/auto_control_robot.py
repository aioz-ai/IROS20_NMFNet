#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import time


def auto_control():

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('auto_control')
    rate = rospy.Rate(10) #1Hz = 0.1s => 1s = 10Hz
    # while(1):

    for i in range(50): # 100 = 0.1*100 = 10s
        twist = Twist()
        twist.linear.x = 6
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)
        rate.sleep()
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)


if __name__ == '__main__':
    try:
        auto_control()
    except rospy.ROSInterruptException:
        pass
