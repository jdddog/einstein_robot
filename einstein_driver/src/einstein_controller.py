#!/usr/bin/env python
__author__ = 'Jamie Diprose'

import rospy
from sensor_msgs.msg import JointState
from ros_pololu_servo.msg import servo_pololu
import math


class EinsteinController():
    def __init__(self):
        rospy.init_node('einstein_controller')
        rospy.Subscriber("joint_angles", JointState, self.handle_joint_angles, queue_size=10)
        self.pololu_pub = rospy.Publisher("cmd_pololu", servo_pololu)
        self.joint_ids = {'neck_yaw': 23, 'neck_roll': 2, 'neck_pitch': 3}

    def handle_joint_angles(self, msg):
        rospy.logdebug("Received a joint angle target")

        for i, joint_name in enumerate(msg.name):
            servo_msg = servo_pololu()
            servo_msg.id = self.joint_ids[joint_name]
            servo_msg.angle = msg.position[i]
            servo_msg.speed = (msg.velocity * 255.0)
            servo_msg.acceleration = msg.effort #TODO: check this
            self.pololu_pub.publish(servo_msg)

            #tTODO: enforce joint angles

if __name__ == '__main__':
    rospy.loginfo("Starting einstein_controller...")
    controller = EinsteinController()
    controller.start()
    rospy.loginfo("einstein_controller started")

    rospy.spin()

    rospy.loginfo("einstein_controller stopped")