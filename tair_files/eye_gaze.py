#!/usr/bin/env python

"""A program to perform gaze movements with Sawyer Robot"""
import sys
from intera_interface import HeadDisplay
import rospy
from std_msgs.msg import Int32, String
import time


epilog = """
Notes:
    Max screen resolution is 1024x600.
    Images are always aligned to the top-left corner.
    Image formats are those supported by OpenCv - LoadImage().
    """

class SawyerGazeController:

    def __init__(self):
        # #########################Fields#####################################
        self.head_display = HeadDisplay()
        self.eyegaze_down_files_location = "/home/sawyer/tair_catkin_ws/src/tair_files/Eyes_Down/Comp 1_"
        self.eyegaze_up_files_location = "/home/sawyer/tair_catkin_ws/src/tair_files/Eyes_Up/Comp 1_"
        self.handover_phase = 'home'
        # ################### Subscribers ####################################
        # Topic for getting handover phase status
        rospy.Subscriber('/handover_phase_status', String, self.handover_phase_status_callback)

    # ################### Publishers ####################################

        # ################### Actions ######################################

        # ################### Services ####################################

        # ################## Startup Procedure ############################

    # ################### Action Clients ###################################
    # def actionClient():

    # ################### Service Clients ##################################
    # def ServiceClient():

    # ################### Callback Functions ###############################
    def handover_phase_status_callback(self, msg):
        print(msg.data)
        self.handover_phase = msg.data
    # ################### Methods ###########################################

    def face_hand_transition_gaze(self):
        images = []
        for i in range(0, 30):
            path = self.eyegaze_down_files_location + str(i) + ".jpg"
            images.append(path)
        # print(images)
        self.head_display.display_image(images, False, 30)

    def hand_face_transition_gaze(self):
        images = []
        for i in range(90, 120):
            path = self.eyegaze_up_files_location + str(i) + ".jpg"
            images.append(path)
        # print(images)
        self.head_display.display_image(images, False, 30)

    def face_gaze(self):
        path = self.eyegaze_down_files_location + str(0) + ".jpg"
        self.head_display.display_image(path, False, 30)

    def hand_gaze(self):
        path = self.eyegaze_up_files_location + str(90) + ".jpg"
        self.head_display.display_image(path, False, 30)

    def hand_face_gaze(self):
        # First gaze condition
        # hand-face transition after the transfer phase
        while(self.handover_phase != 'retreat'):
            print(self.handover_phase)
            self.hand_gaze()
        self.hand_face_transition_gaze()

    def face_hand_face_gaze(self):
        # Second gaze condition
        # face-hand transition after 0.5 reach phase
        # hand-face transition after 0.5 retreat phase
        while (self.handover_phase == 'home'):
            print(self.handover_phase)
            self.face_gaze()

        transition_start = time.time() + 1.0

        while(self.handover_phase == 'reach'):
            while(time.time() < transition_start):
                self.face_gaze()
            self.face_hand_transition_gaze()
            while (self.handover_phase != 'retreat'):
                self.hand_gaze()


        transition_start = time.time() + 1.0

        while(self.handover_phase == 'retreat'):
            while(time.time() < transition_start):
                self.hand_gaze()
            self.hand_face_transition_gaze()
            while(self.handover_phase == 'retreat'):
                self.face_gaze()



if __name__ == '__main__':
    # Initialize node
    rospy.init_node("eye_gaze", anonymous=True, disable_signals=True)

    try:
        brain = SawyerGazeController()
        print(sys.argv[1])
        if sys.argv[1] == '1':
            brain.hand_face_gaze()
        elif sys.argv[1] == '2':
            brain.face_hand_face_gaze()
        elif sys.argv[1] == '3':
            for i in range(10):
                brain.hand_gaze()
        else:
            print("Incorrect input")
    except rospy.ROSInterruptException:
        pass
