#!/usr/bin/env python3

import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d

import rospy
import moveit_commander
import actionlib
from franka_gripper.msg import MoveGoal, MoveAction
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory, PlanningScene
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list

CONTROLLER_TOPIC = "/position_joint_trajectory_controller/follow_joint_trajectory"
DESIRED_JOINT_STATE_TOPIC = "/joint_states_desired"
VELOCITY_MULTIPLIER = 0.2
MAX_COMMAND_POINT_DIFF = 0.05


class DemoInterface(object):
    """Demo Interface"""
    def __init__(self, node_initialized=False):
        super(DemoInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        if not node_initialized:
            rospy.init_node('demo_interface', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        # self.set_planner_id("RTRRTstarkConfigDefault")
        self.set_planner_id("RRTstarkConfigDefault")
        self.set_planning_time(0.5)
        self.move_group.set_end_effector_link("panda_hand")
        self.prev_command_point = None
        self.simulation = rospy.get_param('/simulation', False)
        if self.simulation:
            rospy.logwarn(f"Running demo in simulation")
        else:
            rospy.logwarn(f"Running demo on hardware")
            self.hardware_setup()

    def hardware_setup(self):
        # Setup joint and gripper controller clients
        self.trajectory_client = actionlib.SimpleActionClient(CONTROLLER_TOPIC,
                                                              FollowJointTrajectoryAction)
        while not self.trajectory_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo(f"Waiting for the {CONTROLLER_TOPIC} action server")
        self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/move',
                                                           MoveAction)
        self.gripper_client.wait_for_server()
        self.close_goal = MoveGoal(width=0.054, speed=0.08)
        self.open_goal = MoveGoal(width=0.08, speed=0.08)

    def set_planner_id(self, planner_id):
        self.move_group.set_planner_id(planner_id)

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
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - float(goal[index])) > tolerance:
                    return False
        elif type(goal) is PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)
        elif type(goal) is Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual),
                                  tolerance)
        return True

    def open_gripper(self, wait=True):
        self.gripper_client.send_goal(self.open_goal)
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def close_gripper(self, wait=True):
        self.gripper_client.send_goal(self.close_goal)
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def plan_to_joint_goal(self, joint_values):
        self.move_group.set_joint_value_target(joint_values)
        # Note returns a tuple: (Success, Trajectory Msg, Planning Time, Error Code)
        return self.move_group.plan()

    def go_to_joint_goal(self, joint_values, wait=True):
        joint_goal = self.move_group.get_current_joint_values()
        for i in range(7):
            joint_goal[i] = joint_values[i]
        self.move_group.go(joint_goal, wait)
        if wait:
            self.smooth_stop()
            current_joints = self.move_group.get_current_joint_values()
            return self.all_close(joint_goal, current_joints, 0.01)
        return True

    def go_to_start(self, wait=True):
        (success, start_plan, planning_time, error_code) = self.plan_to_start()
        self.move_group.execute(start_plan)

    def plan_to_start(self):
        joint_values = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        return self.plan_to_joint_goal(joint_values)

    def follow_point(self, point, grasp=False, wait=True):
        # Stop the robot before planning new path
        self.smooth_stop()
        # Adding object as obstacle so we don't hit it as we approach
        # .055 x .065 x .065
        self.publish_object("goal", point, size=(0.05, 0.06, 0.055), offset=(-0.03, 0, 0))
        pose_goal = Pose()
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
        move_group.go(wait)

    def planning_test(self, point, approach="top"):
        # Adding object as obstacle so we don't hit it as we approach
        self.publish_object("goal", point, size=0.0275, type='sphere')
        pose_goal = Pose()
        move_group = self.move_group
        if approach == "top":
            rpy_rot = R.from_euler('y', 0, degrees=True) * R.from_euler('x', 180, degrees=True)
        elif approach == "front":
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True)
        elif approach == "left":
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True) * R.from_euler('x', 90, degrees=True)
        elif approach == "right":
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True) * R.from_euler('x', -90, degrees=True)
        elif approach == "back":
            theta = 90
            rpy_rot = R.from_euler('y', theta, degrees=True) * R.from_euler('x', 180, degrees=True)
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
        rospy.Subscriber("/point_command", Point, callback=self.check_point_position_diff,
                         queue_size=1)
        rospy.spin()

    def check_point_position_diff(self, point):
        if self.prev_command_point:
            diff = self.calc_point_diff(self.prev_command_point, point)
            if diff > MAX_COMMAND_POINT_DIFF:
                rospy.loginfo("Sending new follow command, goal move detected")
                self.follow_point(point, wait=False)
        else:
            rospy.loginfo("Sending follow command, first goal point received")
            self.follow_point(point, wait=False)
        self.prev_command_point = point

    def calc_point_diff(self, point1, point2):
        x_diff = point2.x - point1.x
        y_diff = point2.y - point1.y
        z_diff = point2.z - point2.z
        return np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)

    def smooth_stop(self):
        if self.simulation:
            rospy.logwarn("Stopping execution (SIMULATION)")
            self.move_group.stop()
        else:
            # might be a good idea to check if we're already stopped
            # before taking the effort to create a stopping msg
            # if self.trajectory_client.simple_state == 2:
            #     return
            rospy.logwarn("Stopping execution (HARDWARE)")
            stop_goal = self.get_stop_goal()
            self.trajectory_client.send_goal_and_wait(stop_goal)

    def get_stop_goal(self):
        goal = FollowJointTrajectoryGoal()
        trajectory_point = JointTrajectoryPoint()
        desired_joint_state = rospy.wait_for_message(DESIRED_JOINT_STATE_TOPIC, JointState)
        positions = desired_joint_state.position
        velocities = desired_joint_state.velocity
        trajectory_point.time_from_start = rospy.Duration(0.5)

        # Fill msg vectors
        for i in range(7):
            # Add joint names
            goal.trajectory.joint_names.append(f"panda_joint{i+1}")
            # Add positions
            trajectory_point.positions.append(positions[i] + (velocities[i] * VELOCITY_MULTIPLIER))
            # Add velocities (ALL 0)
            trajectory_point.velocities.append(0.0)

        goal.trajectory.points.append(trajectory_point)
        return goal
