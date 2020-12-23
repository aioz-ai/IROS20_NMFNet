#!/usr/bin/env python

from __future__ import print_function

import roslib;

roslib.load_manifest('beetlebot_control')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from Levenshtein import *

import sys, select, termios, tty

state = ''
speed = 10
turn = 1320
speed_change = True

msg = """
Reading from the voice and Publishing to Twist!
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
    'GO FORWARD': (1, 0, 0, 0),
    'GO BACKWARD': (-1, 0, 0, 0),
    'TURN LEFT': (1, 0, 0, 1),
    'TURN RIGHT': (1, 0, 0, -1),
    'ROTATION': (-1, 0, 1, 0),  # rotate clockwise
}

speedBindings = {
    'SPEED UP': (1, 0),
    'SLOW DOWN': (-1, 0),
}

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def command_comparision(voice_cmd):
    max_value = 0
    command = ''
    for cmd in moveBindings.keys():
        value = jaro(voice_cmd, cmd)
        if (max_value < value):
            max_value = value
            command = cmd

    for cmd in speedBindings.keys():
        value = jaro(voice_cmd, cmd)
        if (max_value < value):
            max_value = value
            command = cmd
    
    if (max_value < jaro(voice_cmd, 'STOP')):
        max_value = jaro(voice_cmd, 'STOP')
        command = 'STOP'
    
    if (max_value < 0.75):
        return ''
    return command

def command_filter(voice_cmd):
    voice_cmd = str(voice_cmd)
    # Type of voice_cmd: data: "something", filter -> something
    bracket_pos = voice_cmd.find('"')
    voice_cmd = voice_cmd[bracket_pos+1:len(voice_cmd)-1]
    return command_comparision(voice_cmd)

def process(voice_msg):
    global state
    global speed
    global turn
    global speed_change
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    sp = speed
    try:
        print(vels(speed, turn))
        voice_msg = command_filter(voice_msg)
        key = voice_msg
        print(key)
        # if key not in moveBindings.keys() and key != 'STOP':
        #     key = state

        if key in moveBindings.keys():
            th = moveBindings[key][3]
            z = moveBindings[key][2]
            x = moveBindings[key][0]
            if (key == "ROTATE"):
                sp = 3

        elif key in speedBindings.keys():
            if (speed_change is True):
                speed = speed + speedBindings[key][0]
                turn = turn + speedBindings[key][1]
                sp = speed

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                speed_change = False

        elif key == 'STOP':
            x = 0
            y = 0
            z = 0
            th = 0
        
        twist = Twist()
        twist.linear.x = x * sp
        twist.linear.y = y * sp
        twist.linear.z = z * sp
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = th * turn

        pub.publish(twist)

    except Exception as e:
        print(e)

def voice_callback(voice_msg):
    process(voice_msg)


if __name__ == "__main__":
    rospy.init_node('teleop_twist_voice')
    loop_rate = rospy.Rate(1)

    while (not rospy.is_shutdown()):
        speed_change = True
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/io/voice_cmd', String, voice_callback)
        loop_rate.sleep()
