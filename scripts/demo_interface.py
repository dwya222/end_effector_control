#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
from display_marker_publisher import marker_pub


class DemoInterface(object):
    """Demo Interface"""
    def __init__(self):
        super(DemoInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('demo_interface', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_end_effector_link("panda_hand")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=1)
        self.scene_pub = rospy.Publisher('/move_group/monitored_planning_scene',
                                    moveit_msgs.msg.PlanningScene,
                                    queue_size=1)


    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False
        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)
        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
        return True

    def go_to_start(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        rospy.loginfo(joint_goal)
        [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        joint_goal[0] = 0
        joint_goal[1] = -0.785
        joint_goal[2] = 0
        joint_goal[3] = -2.356
        joint_goal[4] = 0
        joint_goal[5] = 1.571
        joint_goal[6] = 0.785
        # setting wait = False allows for dynamic trajectory planning
        move_group.go(joint_goal, wait=False)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, rpy):
        move_group = self.move_group
        print(move_group.get_current_pose())
        print(move_group.get_current_rpy())
        pose_goal = geometry_msgs.msg.Pose()
        current_pose = move_group.get_current_pose()
        rpy_rot = R.from_euler('xyz', rpy, degrees=False)
        quat = rpy_rot.as_quat()
        print(quat)
        pose_goal.position.x = current_pose.pose.position.x
        pose_goal.position.y = current_pose.pose.position.y
        pose_goal.position.z = current_pose.pose.position.z
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        print(move_group.get_current_rpy())
        return self.all_close(pose_goal, current_pose, 0.01)

    def go_to_rpy_goal(self, rpy):
        move_group = self.move_group
        print(move_group.get_current_pose())
        # start_rpy = [3.1415926535848926, -4.896669808048392e-12, -0.785000000006922]
        move_group.set_rpy_target(rpy)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        print("printing current pose")
        print(self.move_group.get_current_pose().pose)
        print("printing current rpy")
        rpy_new = self.move_group.get_current_rpy()
        for num,val in enumerate(rpy_new):
            rpy_new[num] = math.degrees(val)
        print(rpy_new)

    def follow_point(self, point):
        # self.display_point(point)
        self.publish_obstacle(point, (0.03,0.03,0.03))
        print(point)
        x = point.x
        y = point.y
        z = point.z
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()

        theta = self.get_angle(z)
        # The actual point between the grippers is offset by 0.1m
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        print("Theta: %s" % theta)
        print("X motion: %s" % pose_goal.position.x)
        print("Z motion: %s" % pose_goal.position.z)

        rpy_rot = R.from_euler('y', theta, degrees=True) * R.from_euler('x', 180, degrees=True)
        print("RPY Values:")
        print(rpy_rot.as_euler('xyz', degrees=False))
        quat = rpy_rot.as_quat()
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=False)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.01)

    def display_point(self, point):
        marker_publisher = marker_pub()
        marker_publisher.display_marker(point)

    def publish_obstacle(self, point, size):
        # Adding object as obstacle so we don't hit it as we approah
        self.scene_pub = rospy.Publisher('/move_group/monitored_planning_scene',
                                    moveit_msgs.msg.PlanningScene,
                                    queue_size=1)
        obstacle_pose = PoseStamped()
        obstacle_pose.header.frame_id = self.robot.get_planning_frame()
        obstacle_pose.pose.position.x = point.x
        obstacle_pose.pose.position.y = point.y
        obstacle_pose.pose.position.z = point.z
        obstacle_pose.pose.orientation.x = 0.0
        obstacle_pose.pose.orientation.y = 0.0
        obstacle_pose.pose.orientation.z = 0.0
        obstacle_pose.pose.orientation.w = 1.0
        print(obstacle_pose)
        self.scene.add_box("obstacle", obstacle_pose, size=size)

    def listen_for_point(self):
        rospy.Subscriber("/point_command", Point, self.follow_point)
        rospy.spin()

    def get_angle(self, height):
        heights = np.linspace(0, 1, num=20, endpoint=True)
        angles = [-(45 + 90*h) for h in heights]
        f = interp1d(heights, angles)
        return float(f(height))
