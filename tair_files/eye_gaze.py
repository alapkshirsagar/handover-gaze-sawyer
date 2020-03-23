#!/usr/bin/env python

"""A program to perform gaze movements with Sawyer Robot"""

from intera_interface import HeadDisplay
import rospy


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
        self.eyegaze_files_location = "/home/sawyer/tair_catkin_ws/src/tair_files/eyes_new/Untitled-"

        # ################### Publishers ####################################

        # ################### Actions ######################################

        # ################### Services ####################################

        # ################## Startup Procedure ############################

    # ################### Action Clients ###################################
    # def actionClient():

    # ################### Service Clients ##################################
    # def ServiceClient():

    # ################### Callback Functions ###############################

    # ################### Methods ###########################################

    def face_hand_transition_gaze(self):
        images = []
        for i in range(60, 299):
            path = self.eyegaze_files_location + str(i + 10001) + ".jpg"
            images.append(path)
        # print(images)
        self.head_display.display_image(images, True, 30)

    def face_gaze(self):
        path = self.eyegaze_files_location + str(10001) + ".jpg"
        self.head_display.display_image(path, True, 30)

    def hand_gaze(self):
        path = self.eyegaze_files_location + str(10299) + ".jpg"
        self.head_display.display_image(path, True, 30)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node("head_display", anonymous=True, disable_signals=True)

    try:
        brain = SawyerGazeController()
        brain.face_hand_transition_gaze()

    except rospy.ROSInterruptException:
        pass
