#!/usr/bin/env python3

import sys
import copy
import rospy
import genpy
import math
import numpy as np
from math import pi
import moveit_commander
import actionlib
from franka_gripper.msg import MoveGoal, MoveAction
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import (Grasp, GripperTranslation, PlaceLocation,
                             MoveItErrorCodes, RobotTrajectory)
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
from tf.transformations import quaternion_from_euler
from display_marker_publisher import marker_pub
from collections import deque
import time


class DemoInterface(object):
    """Demo Interface"""
    def __init__(self, real=False):
        super(DemoInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('demo_interface', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_planner_id("RRTstarkConfigDefault")
        self.set_planning_time(2.0)
        self.move_group.set_end_effector_link("panda_hand")
        self.display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory,
                                            queue_size=1)
        self.scene_pub = rospy.Publisher('/move_group/monitored_planning_scene',
                                    moveit_msgs.msg.PlanningScene,
                                    queue_size=1)
        # create client and variable for gripper actions
        self.real = real
        if self.real:
            gripper_client = actionlib.SimpleActionClient('/franka_gripper/move',
                                                          MoveAction)
            gripper_client.wait_for_server()
            close_goal = MoveGoal(width = 0.054, speed = 0.08)
            open_goal = MoveGoal(width = 0.08, speed = 0.08)

    def set_planning_time(self, planning_time):
        self.move_group.set_planning_time(planning_time)

    def get_planning_time(self):
        return self.move_group.get_planning_time()

    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a
        tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - float(goal[index])) > tolerance:
                    return False
        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)
        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual),
                                  tolerance)
        return True

    def open_gripper(self, wait=True):
        gripper_client.send_goal(open_goal)
        gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def close_gripper(self, wait=True):
        gripper_client.send_goal(close_goal)
        gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def plan_to_joint_goal(self, joint_values):
        self.move_group.set_joint_value_target(joint_values)
        # Note returns a tuple: (Success, Trajectory Msg, Planning Time, Error Code)
        return self.move_group.plan()

    def go_to_joint_goal(self, joint_values, wait=True):
        joint_goal = self.move_group.get_current_joint_values()
        for i in range(7):
            joint_goal[i] = joint_values[i]
        self.move_group.go(joint_goal, wait)
        self.move_group.stop()
        current_joints = self.move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.01)

    def go_to_start(self, wait=True):
        joint_values = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        return self.go_to_joint_goal(joint_values, wait)

    def plan_to_start(self):
        joint_values = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        return self.plan_to_joint_goal(joint_values)

    def follow_point(self, point, grasp=False):
        # Adding object as obstacle so we don't hit it as we approach
        # .055 x .065 x .065
        self.publish_object("goal", point, size=(0.05,0.06,0.055), offset=(-0.03, 0, 0))
        pose_goal = geometry_msgs.msg.Pose()
        move_group = self.move_group
        if grasp:
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True)
        else:
            theta = self.get_angle(float(point.z))
            rpy_rot = (R.from_euler('y', 0, degrees=True) *
                       R.from_euler('x', 180, degrees=True))
        # Move x point back since we want to focus on the center of the
        # box, but we are given the position of the center of the front
        # side (The box in use has a depth of about 6 cm)
        pose_goal.position.x = float(point.x) - 0.03
        pose_goal.position.y = float(point.y)
        pose_goal.position.z = float(point.z)
        quat = rpy_rot.as_quat()
        pose_goal.orientation.x = float(quat[0])
        pose_goal.orientation.y = float(quat[1])
        pose_goal.orientation.z = float(quat[2])
        pose_goal.orientation.w = float(quat[3])
        current_pose = move_group.get_current_pose().pose
        if self.all_close(pose_goal, current_pose, 0.03):
            return
        move_group.set_pose_target(pose_goal)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        return self.all_close(pose_goal, current_pose, 0.01)

    def planning_test(self, point, approach="top"):
        # Adding object as obstacle so we don't hit it as we approach
        self.publish_object("goal", point, size=0.0275, type='sphere')
        pose_goal = geometry_msgs.msg.Pose()
        move_group = self.move_group
        if approach=="top":
            rpy_rot = (R.from_euler('y', 0, degrees=True) *
                       R.from_euler('x', 180, degrees=True))
        elif approach=="front":
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True)
        elif approach=="left":
            theta = 90
            rpy_rot = (R.from_euler('y', theta, degrees=True) *
                       R.from_euler('x', 90, degrees=True))
        elif approach=="right":
            theta = 90
            rpy_rot = (R.from_euler('y', theta, degrees=True) *
                       R.from_euler('x', -90, degrees=True))
        elif approach=="back":
            theta = 90
            rpy_rot = (R.from_euler('y', theta, degrees=True) *
                       R.from_euler('x', 180, degrees=True))
        else:
            rospy.logerr("Invalid approach parameter. Exiting.")
            return
        pose_goal.position.x = point.x
        pose_goal.position.y = point.y
        pose_goal.position.z = point.z
        quat = rpy_rot.as_quat()
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        move_group.set_pose_target(pose_goal)
        # Note returns a tuple: (Success, Trajectory Msg, Planning Time, Error Code)
        return move_group.plan()

    def get_angle(self, height):
        heights = np.linspace(0, 1, num=20, endpoint=True)
        angles = [-(45 + 90*h) for h in heights]
        f = interp1d(heights, angles)
        return float(f(height))

    def display_points(self, point_list):
        marker_publisher = marker_pub()
        marker_publisher.display_marker(point_list)

    def publish_object_manual(self, name, x, y, z, size, type='box'):
        # May need to sleep for a second before using this after initializing Demo
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose.pose.position.x = float(x)
        object_pose.pose.position.y = float(y)
        object_pose.pose.position.z = float(z)
        object_pose.pose.orientation.x = 0.0
        object_pose.pose.orientation.y = 0.0
        object_pose.pose.orientation.z = 0.0
        object_pose.pose.orientation.w = 1.0
        if type == 'box':
            self.scene.add_box(name, object_pose, size=size)
        else:
            self.scene.add_sphere(name, object_pose, radius=size)


    def publish_object(self, name, point, size, type='box', offset=None, remove=False):
        # May need to sleep for a second before using this after initializing Demo
        if remove:
            self.remove_object(name)
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        # Add x offset as April tag point detection is at the front of box
        if offset is None:
            object_pose.pose.position.x = float(point.x)
            object_pose.pose.position.y = float(point.y)
            object_pose.pose.position.z = float(point.z)
        else:
            object_pose.pose.position.x = float(point.x) + offset[0]
            object_pose.pose.position.y = float(point.y) + offset[1]
            object_pose.pose.position.z = float(point.z) + offset[2]
        object_pose.pose.orientation.x = 0.0
        object_pose.pose.orientation.y = 0.0
        object_pose.pose.orientation.z = 0.0
        object_pose.pose.orientation.w = 1.0
        if type == 'box':
            self.scene.add_box(name, object_pose, size=size)
        else:
            self.scene.add_sphere(name, object_pose, radius=size)

    def remove_object(self, name):
        self.scene.remove_world_object(name)

    def listen_for_point(self):
        # while True:
        # time.sleep(1)
        rospy.Subscriber("/point_command", Point, self.follow_point,
                         queue_size=1)
        rospy.spin()

    def exec_traj(self):
        # Always start this test from the starting position
        self.go_to_start()
        # Setup some local variables to help create RobotTrajectory and
        # JointTrajectoryPoint messages
        current_state = self.move_group.get_current_state().joint_state
        current_joint_positions = current_state.position
        current_joint_velocities = current_state.velocity
        current_joint_efforts = current_state.effort
        # Create our RobotTrajectory message with the appropriate headers
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.header.frame_id = (
                current_state.header.frame_id)
        trajectory.joint_trajectory.joint_names = current_state.name
        # Create 2 points that will be in the RobotTrajectory message
        # (normally there will be  a lot more
        # when these messages are created using move_group.plan() as in the
        # planning_test method in this function
        start_point = JointTrajectoryPoint()
        start_point.positions = current_joint_positions
        start_point.velocities = current_joint_velocities
        start_point.effort = current_joint_efforts
        end_point = copy.deepcopy(start_point)
        end_point_list = list(end_point.positions)
        end_point_list[0] += 0.25
        end_point_list[1] += 0.53
        end_point_list[2] += 0.33
        end_point_list[3] += 0.78
        end_point_list[4] += 0.2
        end_point.positions = end_point_list
        end_point.velocities = [.2, 0, 0, 0, 0, 0, 0, 0, 0]
        end_point.time_from_start = genpy.Duration(2.0)
        points = [start_point, end_point]
        rospy.logwarn("Points: ")
        rospy.loginfo(points)
        trajectory.joint_trajectory.points = points
        self.move_group.execute(trajectory, wait=True)
        rospy.logwarn("Current State: ")
        rospy.loginfo(self.move_group.get_current_state())

    def change_traj(self):
        self.go_to_start()
        point = Point()
        point.x = 0.5
        point.y = 0.2
        point.z = 0.6
        (success, plan, planning_time, error_code) = (
                self.planning_test(point))
        rospy.loginfo("Successfully planned to point 1")
        point2 = Point()
        point2.x = 0.4
        point2.y = -0.3
        point2.z = 0.3

        self.move_group.execute(plan, wait=False)
        initial_exec_time = rospy.Time.now()
        point_time = self.set_plan_state(plan, initial_exec_time)
        (success2, plan2, planning_time2, error_code2) = (
                self.planning_test(point2))
        while True:
            if rospy.Time.now() - initial_exec_time >= point_time:
                # (1) Removal of the following and (2) log statements causes
                # position mismatch failure due to timing issue
                rospy.logwarn("Time from start")
                rospy.loginfo(rospy.Time.now() - initial_exec_time)
                rospy.logwarn("Point time")
                rospy.loginfo(point_time)
                rospy.logwarn("Actual State")
                rospy.loginfo(
                        self.move_group.get_current_state().joint_state.position)
                plan2_exec_time = rospy.Time.now()
                self.move_group.execute(plan2, wait=True)
                break
        self.move_group.set_start_state_to_current_state()
        return plan, plan2, initial_exec_time, plan2_exec_time

    def stop_traj(self):
        self.go_to_start()
        point = Point()
        point.x = 0.5
        point.y = 0.3
        point.z = 0.6
        (success, plan, planning_time, error_code) = self.planning_test(point)

        self.move_group.execute(plan, wait=False)
        rospy.sleep(1)
        self.move_group.stop()


    def set_plan_state(self, plan, initial_exec_time):
        points = plan.joint_trajectory.points
        planning_time = genpy.Duration(self.move_group.get_planning_time())
        current_state = self.move_group.get_current_state()
        future_state = copy.deepcopy(current_state)
        for point in points:
            if point.time_from_start > (planning_time +
                    (rospy.Time.now()-initial_exec_time) + genpy.Duration(1.3)):
                point_time = point.time_from_start
                # (2) Removal of the log statements within this loop  and (1)
                # causes position mismatch failure due to timing issue
                rospy.loginfo(planning_time
                        + (rospy.Time.now()-initial_exec_time))
                future_state.joint_state.position = (list(point.positions) +
                                                     [0.035, 0.035])
                future_state.joint_state.velocity = (list(point.velocities) +
                                                     [0.0, 0.0])
                rospy.loginfo("Predicted state")
                rospy.loginfo(future_state.joint_state.position)
                rospy.loginfo(point.time_from_start)
                self.move_group.set_start_state(future_state)
                return point_time
