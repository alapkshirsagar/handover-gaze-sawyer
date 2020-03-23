#!/usr/bin/env python

"""A program to perform gaze movements with Sawyer Robot"""

from intera_interface import Head
from sensor_msgs.msg import JointState
import rospy


class SawyerHeadMovement:

    def __init__(self):
        # #########################Fields#####################################
        self._head = Head()
        self.base_joint_value = 0
        self.base_start_value = 2.725
        self.pan_joint_value = 0
        # ################### Publishers ####################################

        # ################### Subscribers ###################################
        # Topic for getting joint angles
        rospy.Subscriber('/robot/joint_states', JointState, self.joint_angles_callback)

        # ################### Services ####################################

        # ################## Startup Procedure ############################
        rospy.sleep(2.0)
        while True:
            commanded_value =  -1.2 + self.base_start_value - self.base_joint_value
            print(commanded_value)
            self._head.set_pan(commanded_value)
    # ################### Action Clients ###################################
    # def actionClient():

    # ################### Service Clients ##################################
    # def ServiceClient():

    # ################### Callback Functions ###############################

    # ################### Methods ###########################################
    def joint_angles_callback(self, msg):
        self.base_joint_value = msg.position[1]
        self.pan_joint_value = msg.position[0]


if __name__ == '__main__':
    # Initialize node
    rospy.init_node("head_controller", anonymous=True, disable_signals=True)

    try:
        brain = SawyerHeadMovement()



    except rospy.ROSInterruptException:
        pass
