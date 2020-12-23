#!/usr/bin/env python

from __future__ import print_function

import roslib;

roslib.load_manifest('beetlebot_control')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w    
   a    s    d      

r: rotate right 360 degree
f: rotate right 360 degree
space: stop

i/k : increase/decrease only linear speed by 1
j/l : increase/decrease only turning radius by 10

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    'r': (-1, 0, 1, 0),  # rotate clockwise
    'f': (1, 0, 1, 0) # rotate counter-clockwise
}

speedBindings = {
    'i': (1, 0),
    'k': (-1, 0),
    'j': (0, 10),
    'l': (0, -10),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 10)
    turn = rospy.get_param("~turn", 1320)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while (1):
            key = getKey()

            if key in moveBindings.keys():
                th = moveBindings[key][3]
                z = moveBindings[key][2]

                if key != 'a' and key != 'd':
                    x = moveBindings[key][0]


            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                turn = turn + speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ':
                x = 0
                y = 0
                z = 0
                th = 0
            elif key == '\x03':
                break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
