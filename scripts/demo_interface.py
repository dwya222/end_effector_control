#!/usr/bin/env python

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
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, RobotTrajectory
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
        self.move_group.set_planning_time(1.0)
        self.move_group.set_end_effector_link("panda_hand")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=1)
        self.scene_pub = rospy.Publisher('/move_group/monitored_planning_scene',
                                    moveit_msgs.msg.PlanningScene,
                                    queue_size=1)
        # create client and variable for gripper actions
        self.real = real
        if self.real:
            gripper_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
            gripper_client.wait_for_server()
            close_goal = MoveGoal(width = 0.045, speed = 0.08)
            open_goal = MoveGoal(width = 0.08, speed = 0.08)


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

    def open_gripper(self, wait=True):
        gripper_client.send_goal(open_goal)
        gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def close_gripper(self, wait=True):
        gripper_client.send_goal(close_goal)
        gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def go_to_start(self, wait=True):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        rospy.loginfo(joint_goal)
        joint_goal[0] = 0
        joint_goal[1] = -0.785
        joint_goal[2] = 0
        joint_goal[3] = -2.356
        joint_goal[4] = 0
        joint_goal[5] = 1.571
        joint_goal[6] = 0.785
        # setting wait = False allows for dynamic trajectory planning
        move_group.go(joint_goal, wait)
        move_group.stop()
        if self.real:
            self.open_gripper()
        current_joints = move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.01)

    def follow_point(self, point, grasp=False):
        # Adding object as object so we don't hit it as we approach
        self.publish_object(point, (0.055,0.055,0.055))
        x = point.x
        y = point.y
        z = point.z
        rospy.loginfo(point)
        pose_goal = geometry_msgs.msg.Pose()
        move_group = self.move_group
        if grasp:
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True)
        else:
            theta = self.get_angle(z)
            rpy_rot = R.from_euler('y', theta, degrees=True) * R.from_euler('x', 180, degrees=True)
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        quat = rpy_rot.as_quat()
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        current_pose = move_group.get_current_pose().pose
        if self.all_close(pose_goal, current_pose, 0.03):
            return
        move_group.set_pose_target(pose_goal)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        return self.all_close(pose_goal, current_pose, 0.01)

    def planning_test(self, point, approach="front"):
        # Adding object as object so we don't hit it as we approach
        self.publish_object(point, 0.0275, type='sphere')
        x = point.x
        y = point.y
        z = point.z
        pose_goal = geometry_msgs.msg.Pose()
        move_group = self.move_group
        if approach=="front":
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True)
        elif approach=="left":
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True) * R.from_euler('x', 90, degrees=True)
        elif approach=="right":
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True) * R.from_euler('x', -90, degrees=True)
        elif approach=="back":
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True) * R.from_euler('x', 180, degrees=True)
        else:
            rospy.logerr("Invalid approach parameter. Exiting.")
            return
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        quat = rpy_rot.as_quat()
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        move_group.set_pose_target(pose_goal)
        return move_group.plan()

    def get_angle(self, height):
        heights = np.linspace(0, 1, num=20, endpoint=True)
        angles = [-(45 + 90*h) for h in heights]
        f = interp1d(heights, angles)
        return float(f(height))

    def display_points(self, point_list):
        marker_publisher = marker_pub()
        marker_publisher.display_marker(point_list)

    def publish_object(self, point, size, type='box'):
        rospy.sleep(1)
        self.scene.remove_world_object("object")
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose.pose.position.x = point.x
        object_pose.pose.position.y = point.y
        object_pose.pose.position.z = point.z
        object_pose.pose.orientation.x = 0.0
        object_pose.pose.orientation.y = 0.0
        object_pose.pose.orientation.z = 0.0
        object_pose.pose.orientation.w = 1.0
        if type == 'box':
            self.scene.add_box("object", object_pose, size=size)
        else:
            self.scene.add_sphere("object", object_pose, radius=size)

    def listen_for_point(self):
        # while True:
        # time.sleep(1)
        rospy.Subscriber("/point_command", Point, self.follow_point, queue_size=1)
        rospy.spin()

    def exec_traj(self):
        # Always start this test from the starting position
        self.go_to_start()
        # Setup some local variables to help create RobotTrajectory and JointTrajectoryPoint messages
        current_state = self.move_group.get_current_state().joint_state
        current_joint_positions = current_state.position
        current_joint_velocities = current_state.velocity
        current_joint_efforts = current_state.effort
        # Create our RobotTrajectory message with the appropriate headers
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.header.frame_id = current_state.header.frame_id
        trajectory.joint_trajectory.joint_names = current_state.name
        # Create 2 points that will be in the RobotTrajectory message (normally there will be  a lot more
        # when these messages are created using move_group.plan() as in the planning_test method in this function
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
        plan = self.planning_test(point)
        point2 = Point()
        point2.x = 0.4
        point2.y = -0.3
        point2.z = 0.3

        self.move_group.execute(plan, wait=False)
        initial_exec_time = rospy.Time.now()
        self.set_plan_state(plan, initial_exec_time)
        plan2 = self.planning_test(point2)
        while True:
            if rospy.Time.now() - initial_exec_time >= self.point_time:
                rospy.logwarn("Time from start")
                rospy.loginfo(rospy.Time.now() - initial_exec_time)
                rospy.logwarn("Point time")
                rospy.loginfo(self.point_time)
                rospy.logwarn("Actual State")
                rospy.loginfo(self.move_group.get_current_state().joint_state.position)
                self.move_group.execute(plan2, wait=True)
                break
        # rospy.logwarn("current state:")
        # rospy.loginfo(self.move_group.get_current_state().joint_state.position)
        # rospy.logwarn("current state after execution attempt:")
        # rospy.loginfo(self.move_group.get_current_state().joint_state.position)
        self.move_group.set_start_state_to_current_state()
        # rospy.logwarn("Position list from 2nd plan")
        # for pnt in plan2.joint_trajectory.points:
        #     rospy.logwarn("Pnt:")
        #     rospy.loginfo(pnt)

    def set_plan_state(self, plan, initial_exec_time):
        points = plan.joint_trajectory.points
        planning_time = genpy.Duration(self.move_group.get_planning_time())
        current_state = self.move_group.get_current_state()
        future_state = copy.deepcopy(current_state)
        for point in points:
            if point.time_from_start > (planning_time + (rospy.Time.now()-initial_exec_time) + genpy.Duration(1.3)):
                self.point_time = point.time_from_start
                rospy.loginfo(planning_time + (rospy.Time.now()-initial_exec_time))
                future_state.joint_state.position = list(point.positions) + [0.035,0.035]
                future_state.joint_state.velocity = list(point.velocities) + [0,0]
                rospy.logwarn("Predicted state")
                rospy.loginfo(future_state.joint_state.position)
                rospy.loginfo(point.time_from_start)
                self.move_group.set_start_state(future_state)
                return
