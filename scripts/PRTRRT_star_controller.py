#!/usr/bin/env python3
"""
Parallelized Real-Time RRT* Controller
"""
from abc import abstractmethod
from enum import Enum
from threading import RLock
import numpy as np

import rospy
import actionlib
from std_msgs.msg import Float64MultiArray
from robo_demo_msgs.msg import JointTrajectoryPointStamped, JointTrajectoryPointClearStamped
from moveit_msgs.msg import (ExecuteTrajectoryAction, ExecuteTrajectoryGoal,
                             ExecuteTrajectoryActionResult)
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal,
                              FollowJointTrajectoryActionResult)
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

HW_CONTROLLER_TOPIC = "/position_joint_trajectory_controller/follow_joint_trajectory"
HW_CONTROLLER_RESULT_TOPIC = HW_CONTROLLER_TOPIC + "/result"
SIM_CONTROLLER_TOPIC = "/execute_trajectory"
SIM_CONTROLLER_RESULT_TOPIC = SIM_CONTROLLER_TOPIC + "/result"
PANDA_JOINT_NAMES = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                     'panda_joint5', 'panda_joint6', 'panda_joint7']


class ControllerState(Enum):
    WAIT_FOR_PATH = 0
    WAIT_FOR_EDGE_CLEAR = 1
    EXECUTING = 2


class PRTRRTstarController():

    def __init__(self):
        self.joint_names = PANDA_JOINT_NAMES
        self.control_dur = rospy.get_param('/control_dur', 1.0)
        self.controller_active = False
        self.current_path_mutex = RLock()
        self.edge_clear_mutex = RLock()
        self.new_goal_mutex = RLock()
        self.controller_result_mutex = RLock()
        self.current_path = []
        self.current_goal = []
        self.edge_clear = False
        self.edge_clear_point = None
        self.received_new_goal = False
        self.controller_state = ControllerState.WAIT_FOR_PATH
        self._new_current_path = None
        self._new_edge_clear_msg = None
        self._new_goal_msg = None
        self._new_controller_result_msg = None
        self.setup_controller()
        self.init_subs_pubs()

    def setup_controller(self):
        self.trajectory_client = actionlib.SimpleActionClient(self.controller_topic,
                                                              self.controller_msg_type)
        while not self.trajectory_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo_throttle(1.0, f"Waiting for the {self.controller_topic} action server")
        rospy.loginfo(f"Controller connected. Topic: {self.controller_topic}")

    def init_subs_pubs(self):
        self.current_path_sub = rospy.Subscriber("/current_path", JointTrajectory,
                                                 self.current_path_cb, queue_size=1)
        self.edge_clear_sub = rospy.Subscriber("/edge_clear", JointTrajectoryPointClearStamped,
                                               self.edge_clear_cb, queue_size=3)
        self.new_goal_sub = rospy.Subscriber("/new_planner_goal", Float64MultiArray,
                                             self.new_goal_cb, queue_size=1)
        self.controller_result_sub = rospy.Subscriber(self.controller_result_topic,
                                                      self.controller_result_msg_type,
                                                      self.controller_result_cb, queue_size=1)
        self.executing_to_state_pub = rospy.Publisher("/executing_to_state",
                                                      JointTrajectoryPointStamped, queue_size=1)
        rospy.loginfo(f"Publisher and subscribers initialized")

    def current_path_cb(self, current_path_msg):
        rospy.loginfo("PRTRRTstar Controller received path msg")
        with self.current_path_mutex:
            self._new_current_path = current_path_msg

    def edge_clear_cb(self, point_clear_msg):
        rospy.loginfo("PRTRRTstar Controller received edge clear msg")
        with self.edge_clear_mutex:
            self._new_edge_clear_msg = point_clear_msg

    def new_goal_cb(self, new_goal_msg):
        rospy.loginfo("PRTRRTstar Controller received new goal msg")
        with self.new_goal_mutex:
            self._new_goal_msg = new_goal_msg

    def controller_result_cb(self, result_msg):
        rospy.loginfo("PRTRRTstar Controller received controller result")
        with self.controller_result_mutex:
            self._new_controller_result_msg = result_msg

    def run(self):
        while not rospy.is_shutdown():
            self.handle_callbacks()
            if self.controller_state == ControllerState.WAIT_FOR_PATH:
                if self.current_path:
                    rospy.logwarn("Controller received path, waiting for edge clear")
                    self.controller_state = ControllerState.WAIT_FOR_EDGE_CLEAR
            elif self.controller_state == ControllerState.WAIT_FOR_EDGE_CLEAR:
                if self.received_new_goal:
                    rospy.logwarn("Controller received new goal, waiting for new path")
                    self.controller_state = ControllerState.WAIT_FOR_PATH
                    self.received_new_goal = False
                    continue
                # There are 3 potential outcomes here:
                # 1: waiting for the determination on the next edge
                if not self.next_edge_checked():
                    continue
                # 2: next edge is clear
                elif self.next_edge_clear():
                    self.initiate_next_move()
                    rospy.logwarn("Controller received edge clear, executing control")
                    self.controller_state = ControllerState.EXECUTING
                # 3: next edge is not clear
                else:
                    self.current_path.clear()
                    rospy.logwarn("Controller received edge NOT clear, waiting for new path")
                    self.controller_state = ControllerState.WAIT_FOR_PATH
            elif self.controller_state == ControllerState.EXECUTING:
                if not self.controller_active and len(self.current_path) == 1:
                    rospy.logwarn("Robot controlled to goal state, awaiting next path")
                    self.current_path.clear()
                    self.controller_state = ControllerState.WAIT_FOR_PATH
                elif not self.controller_active:
                    rospy.logwarn("Robot controlled to next state, awaiting next edge clear")
                    self.controller_state = ControllerState.WAIT_FOR_EDGE_CLEAR

    def handle_callbacks(self):
        with self.new_goal_mutex:
            if self._new_goal_msg:
                rospy.loginfo("handling new goal msg")
                self.handle_new_goal()
                self._new_goal_msg = None
        with self.current_path_mutex:
            if self._new_current_path:
                rospy.loginfo("handling new current path")
                self.handle_new_current_path()
                self._new_current_path = None
        with self.edge_clear_mutex:
            if self._new_edge_clear_msg:
                rospy.loginfo("handling new edge clear")
                self.handle_new_edge_clear()
                self._new_edge_clear_msg = None
        with self.controller_result_mutex:
            if self._new_controller_result_msg:
                rospy.loginfo("handling new controller result")
                self.handle_new_controller_result()
                self._new_controller_result_msg = None

    def handle_new_current_path(self):
        self.current_path = self._new_current_path.points

    def handle_new_edge_clear(self):
        self.edge_clear = self._new_edge_clear_msg.clear
        self.edge_clear_point = self._new_edge_clear_msg.trajectory_point

    def handle_new_goal(self):
        self.current_goal = self._new_goal_msg.data
        self.received_new_goal = True
        self.current_path.clear()

    def handle_new_controller_result(self):
        if self._new_controller_result_msg.status.status == 3:
            self.controller_active = False
        else:
            rospy.logerr(f"Unexpected result message status. message: \n{result_msg}")
            raise Exception('Throwing exception due to unexpected controller result')

    def current_path_solution(self):
        """ Check if the last position in the current path is a solution

        returns:
            True if the current path exists and is a solution, otherwise
                returns False
        """
        if len(self.current_path) == 0:
            return False
        if len(self.current_goal) == 0:
            # If we haven't received a changed goal, then we can assume
            # that the existence of a current_path means the
            # current_path is a solution
            return True
        return np.allclose(self.current_path[-1].positions, self.current_goal, atol=0.01)

    def next_edge_clear(self):
        """ Check if next edge in current path is clear

        returns:
            True if the next edge has been cleared, False otherwise
        """
        with self.edge_clear_mutex:
            if len(self.current_path) <= 1:
                rospy.logwarn("Current path too short in next_edge_clear, returning False\n "
                              f"current_path: {self.current_path}")
                return False
            return self.edge_clear and self.next_edge_checked()

    def next_edge_checked(self):
        with self.edge_clear_mutex:
            return self.current_path[1] == self.edge_clear_point

    def initiate_next_move(self):
        joint_position_msg = JointTrajectoryPointStamped()
        self.current_path[1].time_from_start = rospy.Duration(self.control_dur)
        joint_position_msg.trajectory_point = self.current_path[1]
        joint_position_msg.header.stamp = rospy.Time.now()
        self.executing_to_state_pub.publish(joint_position_msg)
        self.initiate_control()
        self.current_path.pop(0)
        self.edge_clear = False
        rospy.loginfo(f"initiate_next_move edge_clear: {self.edge_clear}")

    @abstractmethod
    def initiate_control(self):
        pass


class PRTRRTstarHwController(PRTRRTstarController):

    def __init__(self):
        self.controller_topic = HW_CONTROLLER_TOPIC
        self.controller_msg_type = FollowJointTrajectoryAction
        self.controller_result_topic = HW_CONTROLLER_RESULT_TOPIC
        self.controller_result_msg_type = FollowJointTrajectoryActionResult
        super().__init__()
        rospy.loginfo("Parallelized RT-RRT* HARDWARE controller initialized")

    def initiate_control(self):
        goal_point = self.current_path[1]
        trajectory_msg = FollowJointTrajectoryGoal()
        trajectory_msg.trajectory.joint_names = self.joint_names
        trajectory_msg.trajectory.points.append(goal_point)
        self.trajectory_client.send_goal(trajectory_msg)
        self.controller_active = True


class PRTRRTstarSimController(PRTRRTstarController):

    def __init__(self):
        self.controller_topic = SIM_CONTROLLER_TOPIC
        self.controller_msg_type = ExecuteTrajectoryAction
        self.controller_result_topic = SIM_CONTROLLER_RESULT_TOPIC
        self.controller_result_msg_type = ExecuteTrajectoryActionResult
        super().__init__()
        rospy.loginfo("Parallelized RT-RRT* SIMULATION controller initialized")

    def initiate_control(self):
        current_point = self.current_path[0]
        goal_point = self.current_path[1]
        trajectory_msg = ExecuteTrajectoryGoal()
        trajectory_msg.trajectory.joint_trajectory.joint_names = self.joint_names
        current_point.time_from_start = rospy.Duration(0.0)
        trajectory_msg.trajectory.joint_trajectory.points.extend([current_point, goal_point])
        self.trajectory_client.send_goal(trajectory_msg)
        self.controller_active = True


if __name__ == "__main__":
    rospy.init_node("PRTRRT_star_controller")
    if rospy.get_param('/simulation', False):
        controller = PRTRRTstarSimController()
    else:
        controller = PRTRRTstarHwController()
    controller.run()
