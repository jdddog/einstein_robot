#!/usr/bin/env python
__author__ = 'Jamie Diprose'

import rospy
from sensor_msgs.msg import JointState
from threading import Thread
from ros_pololu_servo.srv import pololu_state


class EinsteinSensors(Thread):
    def __init__(self):
        rospy.init_node('einstein_sensors')

        Thread.__init__(self)
        self.rate = rospy.Rate(rospy.get_param('~sensor_rate', 15.0))
        self.base_frame_id = rospy.get_param('~base_frame_id', "base_link")

        self.joint_state = JointState()
        self.joint_state.name = ['neck_yaw', 'neck_roll', 'neck_pitch']
        self.joint_state_pub = rospy.Publisher("joint_states", JointState)

        rospy.loginfo("Waiting for pololu_status service...")
        self.pololu_status_srv = rospy.ServiceProxy('pololu_status', pololu_state)
        self.pololu_status_srv.wait_for_service()
        rospy.loginfo("pololu_status service found")

    def run(self):
        while not rospy.is_shutdown():
            timestamp = rospy.Time.now()

            self.joint_state.position[0] = self.pololu_status_srv(23)
            self.joint_state.position[1] = self.pololu_status_srv(2)
            self.joint_state.position[2] = self.pololu_status_srv(3)

            self.joint_state.header.stamp = timestamp
            self.joint_state.header.frame_id = self.base_frame_id
            self.joint_state_pub.publish(self.joint_state)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.loginfo("Starting einstein_sensors...")
    sensors = EinsteinSensors()
    sensors.start()
    rospy.loginfo("einstein_sensors started")

    rospy.spin()

    rospy.loginfo("Stopping einstein_sensors...")
    sensors.join()
    rospy.loginfo("einstein_sensors stopped.")