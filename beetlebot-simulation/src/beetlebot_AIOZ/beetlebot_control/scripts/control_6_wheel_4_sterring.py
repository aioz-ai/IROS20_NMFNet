#!/usr/bin/env python
import time
import math
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from robot import RobotCalculator


class DeliveryRobot(object):
    def __init__(self):
        rospy.loginfo("CuriosityRoverAckerMan Initialising...")

        self.calculator = RobotCalculator(
            (19.6, 25.1, 25.1, 19.6), (math.pi / 180, 1.0472))  # (1 degree, 60 degree)
        self.publishers_curiosity_d = {}
        self.controller_ns = "beetlebot"
        self.controller_command = "command"
        self.controllers_list = ["back_wheel_L_joint_velocity_controller",
                                "back_wheel_R_joint_velocity_controller",
                                "front_wheel_L_joint_velocity_controller",
                                "front_wheel_R_joint_velocity_controller",
                                "middle_wheel_L_joint_velocity_controller",
                                "middle_wheel_R_joint_velocity_controller",
                                "sterring_B_L_joint_position_controller",
                                "sterring_B_R_joint_position_controller",
                                "sterring_F_L_joint_position_controller",
                                "sterring_F_R_joint_position_controller"]

        for controller_name in self.controllers_list:
            topic_name = "/"+self.controller_ns+"/" + \
                controller_name+"/"+self.controller_command
            self.publishers_curiosity_d[controller_name] = rospy.Publisher(
                topic_name,
                Float64,
                queue_size=1)

        self.wait_publishers_to_be_ready()
        self.init_publisher_variables()
        self.init_state()

        self.cmd_vel_msg = Twist()
        cmd_vel_topic = "/cmd_vel"
        rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)

        rospy.logwarn("beetlebot_AckerMan...READY")

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def wait_publishers_to_be_ready(self):

        rate_wait = rospy.Rate(10)
        for controller_name, publisher_obj in self.publishers_curiosity_d.iteritems():
            publisher_ready = False
            while not publisher_ready:
                rospy.logwarn("Checking Publisher for ==>" + str(controller_name))
                pub_num = publisher_obj.get_num_connections()
                publisher_ready = (pub_num > 0)
                rate_wait.sleep()
            rospy.loginfo("Publisher ==>" + str(controller_name) + "...READY")

    def init_publisher_variables(self):
        """
        We create variables for more pythonic access access to publishers
        and not need to access any more
        :return:
        """
        # Get the publishers for wheel speed
        self.back_wheel_L = self.publishers_curiosity_d[self.controllers_list[0]]
        self.back_wheel_R = self.publishers_curiosity_d[self.controllers_list[1]]
        self.front_wheel_L = self.publishers_curiosity_d[self.controllers_list[2]]
        self.front_wheel_R = self.publishers_curiosity_d[self.controllers_list[3]]
        self.middle_wheel_L = self.publishers_curiosity_d[self.controllers_list[4]]
        self.middle_wheel_R = self.publishers_curiosity_d[self.controllers_list[5]]
        # Get the publishers for steering
        self.suspension_steer_B_L = self.publishers_curiosity_d[self.controllers_list[6]]
        self.suspension_steer_B_R = self.publishers_curiosity_d[self.controllers_list[7]]
        self.suspension_steer_F_L = self.publishers_curiosity_d[self.controllers_list[8]]
        self.suspension_steer_F_R = self.publishers_curiosity_d[self.controllers_list[9]]

        # Init Messages
        self.back_wheel_L_velocity_msg = Float64()
        self.back_wheel_R_velocity_msg = Float64()
        self.front_wheel_L_velocity_msg = Float64()
        self.front_wheel_R_velocity_msg = Float64()
        self.middle_wheel_L_velocity_msg = Float64()
        self.middle_wheel_R_velocity_msg = Float64()
        self.suspension_steer_B_L_pos_msg = Float64()
        self.suspension_steer_B_R_pos_msg = Float64()
        self.suspension_steer_F_L_pos_msg = Float64()
        self.suspension_steer_F_R_pos_msg = Float64()

    def init_state(self):
        self.set_turning_radius(None, False)
        self.set_wheels_speed(0.0, None, False)

    def set_turning_radius(self, turn_radius, isRotate):

        r_angle = self.calculator.calculateDegree(turn_radius, isRotate)

        self.suspension_steer_B_L_pos_msg.data = r_angle['3']
        self.suspension_steer_B_R_pos_msg.data = r_angle['6']
        self.suspension_steer_F_L_pos_msg.data = r_angle['1']
        self.suspension_steer_F_R_pos_msg.data = r_angle['4']

        self.suspension_steer_B_L.publish(self.suspension_steer_B_L_pos_msg)
        self.suspension_steer_B_R.publish(self.suspension_steer_B_R_pos_msg)
        self.suspension_steer_F_L.publish(self.suspension_steer_F_L_pos_msg)
        self.suspension_steer_F_R.publish(self.suspension_steer_F_R_pos_msg)

    def set_wheels_speed(self, turning_speed, turning_radius, isRotate):
        velocity = self.calculator.calculateVelocity(
            turning_speed, turning_radius, isRotate)

        self.back_wheel_L_velocity_msg.data = velocity['3']
        self.back_wheel_R_velocity_msg.data = velocity['6']
        self.front_wheel_L_velocity_msg.data = velocity['1']
        self.front_wheel_R_velocity_msg.data = velocity['4']
        self.middle_wheel_L_velocity_msg.data = velocity['2']
        self.middle_wheel_R_velocity_msg.data = velocity['5']

        self.back_wheel_L.publish(self.back_wheel_L_velocity_msg)
        self.back_wheel_R.publish(self.back_wheel_R_velocity_msg)
        self.front_wheel_L.publish(self.front_wheel_L_velocity_msg)
        self.front_wheel_R.publish(self.front_wheel_R_velocity_msg)
        self.middle_wheel_L.publish(self.middle_wheel_L_velocity_msg)
        self.middle_wheel_R.publish(self.middle_wheel_R_velocity_msg)

    def move_with_cmd_vel(self):
        wheel_speed = self.cmd_vel_msg.linear.x
        turning_radius = self.cmd_vel_msg.angular.z
        # Check if delivery-robot want to rotate
        isRotate = True if self.cmd_vel_msg.linear.z != 0 else False

        if turning_radius == 0.0:
            turning_radius = None

        rospy.logdebug("turning_radius="+str(turning_radius) + ",wheel_speed="+str(wheel_speed))
        self.set_turning_radius(turning_radius, isRotate)
        self.set_wheels_speed(wheel_speed, turning_radius, isRotate)


if __name__ == "__main__":
    rospy.init_node("beetlebot_AckerMan_node", log_level=rospy.INFO)
    delivery_robot = DeliveryRobot()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        delivery_robot.move_with_cmd_vel()
        rate.sleep()
