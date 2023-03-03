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
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool

CONTROLLER_TOPIC = "/position_joint_trajectory_controller/follow_joint_trajectory"
DESIRED_JOINT_STATE_TOPIC = "/joint_states_desired"
VELOCITY_MULTIPLIER = 0.2
MAX_COMMAND_POINT_DIFF = 0.05
START_JOINT_VALUES = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
# Size of goal box
GOAL_SIZE = (0.05, 0.06, 0.055)
# Offset to account for undetected object depth as the camera detects
# a point on the front surface of the goal box
(GOAL_OFFSET_X, GOAL_OFFSET_Y, GOAL_OFFSET_Z) = (-0.03, 0, 0)
DEFAULT_PLANNING_TIME = 0.5


class DemoInterface(object):
    """Demo Interface"""
    def __init__(self, node_initialized=False):
        if not node_initialized:
            rospy.init_node('demo_interface', anonymous=True)
        self.set_parameters()
        self.setup_moveit()
        self.set_ee_approach_dict()
        self.prev_goal_point = None
        # self.return_first_solution_pub = rospy.Publisher('/return_first_solution', Bool,
        #                                                  queue_size=1, latch=True)
        # self.return_first_solution_pub.publish(Bool(False))
        rospy.set_param("return_first_solution", False)
        if self.simulation:
            rospy.logwarn("Running demo in simulation")
        else:
            rospy.logwarn("Running demo on hardware")
            self.create_hardware_controller_clients()

    def set_parameters(self):
        self.group_name = rospy.get_param('/group_name', "panda_arm")
        self.planner_id = rospy.get_param('/planner_id', "RRTstarkConfigDefault")
        self.simulation = rospy.get_param('/simulation', False)
        self.planning_time = rospy.get_param('/planning_time', DEFAULT_PLANNING_TIME)
        self.end_effector_link = rospy.get_param('/end_effector_link', "panda_hand")
        self.goal_object_topic = rospy.get_param('/goal_object_topic', '/goal_object_position')
        self.start_joint_values = START_JOINT_VALUES
        self.goal_size = GOAL_SIZE
        self.goal_offset = Point(GOAL_OFFSET_X, GOAL_OFFSET_Y, GOAL_OFFSET_Z)

    def setup_moveit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.set_planner_id(self.planner_id)
        self.set_planning_time(self.planning_time)
        self.move_group.set_end_effector_link(self.end_effector_link)

    def set_planner_id(self, planner_id):
        self.move_group.set_planner_id(planner_id)

    def set_planning_time(self, planning_time):
        self.move_group.set_planning_time(planning_time)

    @property
    def get_planning_time(self):
        return self.move_group.get_planning_time()

    def set_ee_approach_dict(self):
        rpy_rot_y90 = R.from_euler('y', 90, degrees=True)
        self.ee_approach_dict = {
                "top": R.from_euler('y', 0, degrees=True) * R.from_euler('x', 180, degrees=True),
                "front": rpy_rot_y90,
                "back": rpy_rot_y90 * R.from_euler('x', 180, degrees=True),
                "left": rpy_rot_y90 * R.from_euler('x', 90, degrees=True),
                "right": rpy_rot_y90 * R.from_euler('x', -90, degrees=True)
                }

    def create_hardware_controller_clients(self):
        self.trajectory_client = actionlib.SimpleActionClient(CONTROLLER_TOPIC,
                                                              FollowJointTrajectoryAction)
        while not self.trajectory_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo(f"Waiting for the {CONTROLLER_TOPIC} action server")
        self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/move',
                                                           MoveAction)
        self.gripper_client.wait_for_server()
        self.close_goal = MoveGoal(width=0.054, speed=0.08)
        self.open_goal = MoveGoal(width=0.08, speed=0.08)

    def open_gripper(self, wait=True):
        self.gripper_client.send_goal(self.open_goal)
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def close_gripper(self, wait=True):
        self.gripper_client.send_goal(self.close_goal)
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def plan_to_start(self):
        return self.plan_to_joint_goal(self.start_joint_values)

    def plan_to_joint_goal(self, joint_values, return_first_solution=False):
        """Plan to joint goal.

        Returns:
          Result tuple (bool, RobotTrajectory, float, MoveItErrorCode):
            (success, path, planning time, error code)
        """
        # self.return_first_solution_pub.publish(Bool(return_first_solution))
        rospy.set_param("return_first_solution", return_first_solution)
        self.move_group.set_joint_value_target(joint_values)
        return self.move_group.plan()

    def plan_to_point(self, point, approach="top"):
        """Plan to point goal with specified end-effector approach.

        Returns:
          Result tuple (bool, RobotTrajectory, float, MoveItErrorCode):
            (success, path, planning time, error code)

        Raises:
          KeyError: If invalid approach arg provided.
        """
        # Adding goal object to scene so we plan around it
        self.publish_goal_object(point)
        pose_goal = self.create_grasp_pose_msg(point, approach=approach)
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.plan()

    def create_grasp_pose_msg(self, point, approach="top"):
        """Create pose msg based on object point and desired approach.

        Args:
          point (Point): Point in space of the object
          approach (str): Descriptor of the desired approach
            orientation of the end effector

        Returns:
          Pose: End effector pose to grasp object at given position and
            desired approach
        """
        pose = Pose()
        if approach == "interpolated":
            theta = self.get_angle(float(point.z))
            rpy_rot = (R.from_euler('y', theta, degrees=True)
                       * R.from_euler('x', 180, degrees=True))
        else:
            rpy_rot = self.ee_approach_dict[approach]
        # Move x point back since we want to focus on the center of the
        # box, but we are given the position of the center of the front
        # side (The box in use has a depth of about 6 cm)
        pose.position = point
        quat = rpy_rot.as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
                quat[0], quat[1], quat[2], quat[3])
        return pose

    def get_angle(self, height):
        heights = np.linspace(0, 1, num=20, endpoint=True)
        angles = [-(45 + 90*h) for h in heights]
        f = interp1d(heights, angles)
        return float(f(height))

    def go_to_start(self, wait=True):
        self.go_to_joint_goal(self.start_joint_values, wait=wait)

    def go_to_joint_goal(self, joint_values, wait=True):
        # Stop the robot before planning & executing new path
        self.smooth_stop()
        self.move_group.go(joint_values, wait)

    def go_to_point(self, point, approach="top", wait=True):
        # Stop the robot before planning & executing new path
        self.smooth_stop()
        # Adding goal object to scene so we plan around it
        self.publish_goal_object(point)
        pose_goal = self.create_grasp_pose_msg(point, approach=approach)
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait)

    def publish_goal_object(self, point):
        self.publish_object("goal", point, size=self.goal_size)

    def publish_object_xyz(self, name, x, y, z, size, primitive='box', remove=False):
        point = Point(x, y, z)
        self.publish_object(name, point, size, primitive=primitive, remove=remove)

    def publish_object(self, name, point, size, primitive='box', remove=False):
        if remove:
            self.remove_object(name)
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.move_group.get_planning_frame()
        object_pose.pose.position = point
        if primitive == 'box':
            self.scene.add_box(name, object_pose, size=size)
        else:
            self.scene.add_sphere(name, object_pose, radius=size)

    def remove_object(self, name):
        self.scene.remove_world_object(name)

    def listen_for_goal(self):
        rospy.Subscriber(self.goal_object_topic, Point, callback=self.filter_detection_noise,
                         queue_size=1)
        rospy.spin()

    def filter_detection_noise(self, goal_point):
        if self.prev_goal_point:
            diff = self.euclidean_distance(self.prev_goal_point, goal_point)
            if diff > MAX_COMMAND_POINT_DIFF:
                rospy.loginfo("Goal point movement detected, attempting new plan")
                goal_point = self.offset_point(goal_point, self.goal_offset)
                self.go_to_point(goal_point, wait=False)
        else:
            rospy.loginfo("First goal point received, attempting to plan")
            goal_point = self.offset_point(goal_point, self.goal_offset)
            self.go_to_point(goal_point, wait=False)
        self.prev_goal_point = goal_point

    def euclidean_distance(self, point1, point2):
        arr1 = np.array((point1.x, point1.y, point1.z))
        arr2 = np.array((point2.x, point2.y, point2.z))
        return np.linalg.norm(arr2 - arr1)

    def offset_point(self, point, offset):
        point_offset = Point()
        point_offset.x = point.x + offset.x
        point_offset.y = point.y + offset.y
        point_offset.z = point.z + offset.z
        return point_offset

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

    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a
        tolerance of their counterparts in another list.

        Args:
          goal (list): A list of goal floats, a Pose or a PoseStamped
          actual (list): A list of floats, a Pose or a PoseStamped
          tolerance (float): Allowed difference between goal and actual
            values

        Returns:
          Bool: Successful if true
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
