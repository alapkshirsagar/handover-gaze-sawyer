#! /usr/bin/env python
"""A program to perform handovers with Sawyer Robot"""

import rospy
import multiprocessing
from libezgripper import create_connection, Gripper
import socket
import time
from std_msgs.msg import Int32
from intera_interface import Limb, HeadDisplay, Head
from intera_motion_msgs.msg import TrajectoryOptions
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from geometry_msgs.msg import PoseStamped
import numpy as np


# Class to control Sawyer robot


class SawyerHandoverController:
    def __init__(self):
        # #########################Fields#####################################
        self.toggle_sensor_reading = 0
        self.range_sensor_reading = 1024
        self.range_sensor_threshold = 300
        self.connection = create_connection(dev_name='/dev/ttyUSB0', baudrate=57600)
        self.gripper = Gripper(self.connection, 'gripper1', [1])
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.limb = Limb()
        self.traj_options = TrajectoryOptions()
        self.head_display = HeadDisplay()

        self.linear_speed = 0.3
        self.linear_accel = 0.3
        self.rotational_speed = 1.57 / 2
        self.rotational_accel = 1.57 / 2
        self.tip_name = "right_hand"
        self.timeout = None
        self.home_position = [-0.1, 0.38, 0.2, 0.5, 0.5, 0.5, -0.5]
        #self.waypoint_handover = [0, 0.58, 0.2, 0.5, 0.5, 0.5, -0.5]
        self.handover_location = [-0.1, 0.8, 0.2, 0.5, 0.5, 0.5, -0.5]
        # ################### Subscribers ####################################
        # Topic for getting toggle sensor reading
        rospy.Subscriber('/toggle_sensor', Int32, self.toggle_sensor_callback)

        # Topic for getting range sensor reading
        rospy.Subscriber('/range_sensor', Int32, self.range_sensor_callback)

        # Topic for getting end effector state
        # ################### Publishers ####################################

        # ################### Actions ######################################

        # ################### Services ####################################

        # ################## Startup Procedure ############################
        self.initialize_gripper()
        self.perform_handover()

    # ################### Action Clients ###################################
    # def actionClient():

    # ################### Service Clients ##################################
    # def ServiceClient():

    # ################### Callback Functions ###############################
    def toggle_sensor_callback(self, msg):
        self.toggle_sensor_reading = msg.data

    def range_sensor_callback(self, msg):
        self.range_sensor_reading = msg.data

    # ################### Methods ###########################################
    def initialize_gripper(self):
        self.gripper.calibrate()
        self.open_gripper()

    def open_gripper(self):
        self.gripper.goto_position(80, 100)  # (Position, Effort)
        print "gripper opened"

    def close_gripper(self):
        self.gripper.goto_position(10, 100)
        print "gripper closed"

    def perform_handover(self):
        # Go to home position
        self.go_to_cartesian_pose(self.home_position)
        # self.go_to_cartesian_pose(self.waypoint_handover)
        # self.go_to_cartesian_pose(self.home_position)
        # self.faceGaze()

        # Open Gripper
        self.open_gripper()
        # self.handGaze()

        # Go to handover location
        while self.range_sensor_reading > self.range_sensor_threshold:
            print(self.range_sensor_reading)
            # print("not here")
            rospy.sleep(0.1)
        else:
            # workerMovement = multiprocessing.Process(target=self.go_to_cartesian_pose, args = [self.handover_location])
            # workerMovement.start()
            # workerEyegaze = multiprocessing.Process(target=self.faceHandTransitionGaze)
            # print("here")
            # workerEyegaze.start()
            # print("also here")
            # workerMovement.join()
            # workerEyegaze.join()
            # print("done")
            # self.follow_way_points(self.waypoint_handover, self.handover_location)
            #self.go_to_cartesian_pose(self.waypoint_handover)
            self.go_to_cartesian_pose(self.handover_location)

            # self.faceHandTransitionGaze()

        # Close gripper
        while self.toggle_sensor_reading is 0:
            rospy.sleep(0.1)
        else:
            self.close_gripper()

        # Go to home position
        self.go_to_cartesian_pose(self.home_position)
        rospy.sleep(2.0)

        # Open Gripper
        self.open_gripper()

    def follow_way_points(self, waypoints1, destination):
        self.go_to_cartesian_pose(waypoints1)
        self.go_to_cartesian_pose(destination)

    def go_to_cartesian_pose(self, target_pose):
        self.traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options=self.traj_options, limb=self.limb)

        wpt_opts = MotionWaypointOptions(max_linear_speed=self.linear_speed,
                                         max_linear_accel=self.linear_accel,
                                         max_rotational_speed=self.rotational_speed,
                                         max_rotational_accel=self.rotational_accel,
                                         max_joint_speed_ratio=1.0)
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self.limb)

        joint_names = self.limb.joint_names()
        print(joint_names)
        endpoint_state = self.limb.tip_state(self.tip_name)
        if endpoint_state is None:
            rospy.logerr('Endpoint state not found with tip name %s', self.tip_name)
            return None
        pose = endpoint_state.pose

        if pose is not None and len(target_pose) == 7:
            pose.position.x = target_pose[0]
            pose.position.y = target_pose[1]
            pose.position.z = target_pose[2]
            pose.orientation.x = target_pose[3]
            pose.orientation.y = target_pose[4]
            pose.orientation.z = target_pose[5]
            pose.orientation.w = target_pose[6]

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        # using current joint angles for null space basis if not provided
        joint_angles = self.limb.joint_ordered_angles()
        waypoint.set_cartesian_pose(pose_stamped, self.tip_name, joint_angles)

        # rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

        traj.append_waypoint(waypoint.to_msg())
        print(target_pose)

        result = traj.send_trajectory(timeout=self.timeout)
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('sawyer_handover_gaze', anonymous=False)

    try:
        brain = SawyerHandoverController()
    except rospy.ROSInterruptException:
        pass
