#!/usr/bin/env python3
"""

This node provides a planning & control interface to allow use of a
static planner (like RRTstar) in a dynamic environment.

Subscribes to the "new_planner_goal" and "obstacles_changed" topics for
dynamic information about the environment.


"""

from abc import abstractmethod
from enum import Enum
import os
import numpy as np
from threading import RLock, Thread

import actionlib
import rospy
import rospkg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from robo_demo_msgs.msg import JointTrajectoryPointStamped, JointTrajectoryPointClearStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import (ExecuteTrajectoryAction, ExecuteTrajectoryGoal,
                             ExecuteTrajectoryActionResult, RobotState)
from sensor_msgs.msg import JointState

from demo_interface import DemoInterface
import utils

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
PLAN_TIME_FILE_PATH = os.path.join(PLANNING_DATA_PATH, 'rrt_last_plan_time.txt')
HW_CONTROLLER_TOPIC = "/position_joint_trajectory_controller/follow_joint_trajectory"
HW_CONTROLLER_RESULT_TOPIC = HW_CONTROLLER_TOPIC + "/result"
SIM_CONTROLLER_TOPIC = "/execute_trajectory"
SIM_CONTROLLER_RESULT_TOPIC = SIM_CONTROLLER_TOPIC + "/result"
PANDA_JOINT_NAMES = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                     'panda_joint5', 'panda_joint6', 'panda_joint7']
LARGE_COST = 1000000.0


class MonitorState(Enum):
    WAIT_FOR_GOAL = 0
    PLANNING = 1
    WAIT_FOR_EDGE_CLEAR = 2
    EXECUTING = 3


class RRTstarMonitor():

    def __init__(self):
        # Set planning process param to inform recorder
        rospy.set_param("/planning_process", "Monitored RRTstar")
        self.robot_interface = DemoInterface(node_initialized=True)
        self.robot_interface.move_group.set_planner_id("RRTstarkConfigRealTimeTesting")
        self.robot_interface.set_planning_time(1.0)
        self.control_dur = rospy.get_param('/control_dur', 1.0)
        self.min_planning_time = rospy.get_param('/minimum_planning_time', 0.1)
        self.control_end_time = rospy.Time.now()
        self.current_state_positions = (
                self.robot_interface.move_group.get_current_state().joint_state.position[0:7])
        self.current_goal = []
        self.current_path = []
        self.current_path_cost = LARGE_COST
        self.edge_clear_point = None
        self.current_path_mutex = RLock()
        self.edge_clear_mutex = RLock()
        self.new_goal_mutex = RLock()
        self.controller_result_mutex = RLock()
        self.return_first_solution = True
        self.received_new_goal = False
        self.controller_active = False
        self.monitor_state = MonitorState.WAIT_FOR_GOAL
        self._new_current_path = None
        self._new_edge_clear_msg = None
        self._new_goal_msg = None
        self._new_controller_result_msg = None
        self.joint_names = PANDA_JOINT_NAMES
        self.setup_controller()
        self.init_subs_pubs()

    def setup_controller(self):
        self.trajectory_client = actionlib.SimpleActionClient(self.controller_topic,
                                                              self.controller_msg_type)
        while not self.trajectory_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo(f"Waiting for the {self.controller_topic} action server")
        rospy.loginfo(f"Controller connected. Topic: {self.controller_topic}")

    def init_subs_pubs(self):
        self.current_path_sub = rospy.Subscriber("/current_path", JointTrajectory,
                                                 self.current_path_cb, queue_size=1)
        self.edge_clear_sub = rospy.Subscriber("/edge_clear", JointTrajectoryPointClearStamped,
                                               self.edge_clear_cb, queue_size=1)
        self.new_goal_sub = rospy.Subscriber("/new_planner_goal", Float64MultiArray,
                                             self.new_goal_cb, queue_size=1)
        self.controller_result_sub = rospy.Subscriber(self.controller_result_topic,
                                                      self.controller_result_msg_type,
                                                      self.controller_result_cb, queue_size=1)
        self.executing_to_state_pub = rospy.Publisher("/executing_to_state",
                                                      JointTrajectoryPointStamped, queue_size=1)
        self.preempt_planner_pub = rospy.Publisher("/preempt_planner", Bool, queue_size=1)

    def current_path_cb(self, current_path_msg):
        rospy.loginfo("RRTstar Monitor received path msg")
        with self.current_path_mutex:
            self._new_current_path = current_path_msg

    def edge_clear_cb(self, point_clear_msg):
        rospy.loginfo("RRTstar Monitor received edge clear msg")
        with self.edge_clear_mutex:
            self._new_edge_clear_msg = point_clear_msg

    def new_goal_cb(self, new_goal_msg):
        rospy.loginfo("RRTstar Monitor received new goal msg")
        rospy.loginfo(f"RRTstar Monitor state: {self.monitor_state}")
        with self.new_goal_mutex:
            self._new_goal_msg = new_goal_msg

    def controller_result_cb(self, result_msg):
        rospy.loginfo("RRTstar Monitor received controller result")
        with self.controller_result_mutex:
            self._new_controller_result_msg = result_msg

    def run(self):
        while not rospy.is_shutdown():
            self.handle_callbacks()
            if self.monitor_state == MonitorState.WAIT_FOR_GOAL:
                # Start planning once we get a new goal
                if self.received_new_goal:
                    self.start_planning_thread(reset_path=True)
                    rospy.logwarn("Monitor received new goal, planning")
                    self.monitor_state = MonitorState.PLANNING
                    self.received_new_goal = False
            elif self.monitor_state == MonitorState.PLANNING:
                # If we get a new goal while planning, preempt and
                # restart planner
                if self.received_new_goal:
                    rospy.logwarn("Monitor received new goal while planning, replanning")
                    self.start_planning_thread(preempt=True)
                    self.received_new_goal = False
                # If we get a current path, which means the planner
                # completed and published, then wait for the next edge
                # in the path to be cleared
                elif self.current_path:
                    self.monitor_state = MonitorState.WAIT_FOR_EDGE_CLEAR
                    rospy.logwarn("Monitor received current_path, waiting for edge clear")
            elif self.monitor_state == MonitorState.WAIT_FOR_EDGE_CLEAR:
                if self.received_new_goal:
                    self.start_planning_thread()
                    rospy.logwarn("Monitor received new goal waiting for edge clear, replanning")
                    self.monitor_state = MonitorState.PLANNING
                    self.received_new_goal = False
                    continue
                # There are 3 potential outcomes here:
                # 1: waiting for the determination on the next edge
                if not self.next_edge_checked():
                    rospy.loginfo_throttle(1.0, "next edge not yet checked")
                    continue
                # 2: next edge is clear
                elif self.next_edge_clear():
                    self.initiate_next_move()
                    rospy.logwarn("Monitor received edge clear, executing control")
                    self.monitor_state = MonitorState.EXECUTING
                # 3: next edge is not clear
                else:
                    rospy.logwarn("Monitor received edge NOT clear, replanning")
                    self.start_planning_thread(reset_path=True)
                    self.monitor_state = MonitorState.PLANNING
            elif self.monitor_state == MonitorState.EXECUTING:
                # Allow the planner to plan and attempt to improve the
                # rest of the current path while control executing
                control_time_left = (self.control_end_time - rospy.Time.now()).to_sec()
                if (not self.planning_thread.is_alive()
                        and control_time_left >= self.min_planning_time):
                    self.start_planning_thread(control_time_left=control_time_left)
                if not self.controller_active and len(self.current_path) == 1:
                    rospy.logwarn("Robot controlled to goal state, awaiting next path")
                    self.monitor_state = MonitorState.WAIT_FOR_GOAL
                elif not self.controller_active:
                    rospy.logwarn("Robot controlled to next state, awaiting next edge clear")
                    self.monitor_state = MonitorState.WAIT_FOR_EDGE_CLEAR

    def handle_callbacks(self):
        with self.current_path_mutex:
            if self._new_current_path:
                rospy.loginfo("handling new current path")
                self.handle_new_current_path()
                self._new_current_path = None
        with self.new_goal_mutex:
            if self._new_goal_msg:
                rospy.loginfo("handling new goal msg")
                self.handle_new_goal()
                self._new_goal_msg = None
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
        """Update current path if the new one is better than the current one

           Note: Also need to make sure that the new path has the correct start state.
        """
        if len(self._new_current_path.points) == 0:
            # TODO: what if planning fails but we still have a valid path from earlier?
            rospy.logerr("Planning failed, waiting for new goal")
            self.monitor_state = MonitorState.WAIT_FOR_GOAL
            return
        if not self.current_state_positions == self._new_current_path.points[0].positions:
            rospy.logerr("New current path start positions mismatch. \ncurrent state positions: "
                         f"{self.current_state_positions}. \n new current path start positions: "
                         f"{self._new_current_path.points[0].positions}")
            return
        if not utils.almost_equal(self.current_goal, self._new_current_path.points[-1].positions):
            rospy.logwarn("New path msg does NOT satisfy goal, not setting to current path")
            rospy.loginfo(f"goal: {self.current_goal}, last in path: "
                          f"{self._new_current_path.points[-1].positions}")
            return
        new_path_cost = utils.path_cost(self._new_current_path.points)
        rospy.loginfo(f"comparing new cost: {new_path_cost} to current: {self.current_path_cost}")
        if new_path_cost < self.current_path_cost:
            rospy.loginfo(f"Received new current path with better cost of {new_path_cost}, setting"
                          " new path")
            self.current_path_cost = new_path_cost
            self.current_path = self._new_current_path.points
        else:
            rospy.logwarn(f"Received new current path with worse cost, keeping old path")

    def handle_new_goal(self):
        self.received_new_goal = True
        self.current_goal = self._new_goal_msg.data
        if self.current_path and not utils.almost_equal(self.current_goal,
                                                       self.current_path[-1].positions):
            rospy.logwarn("Clearing current path, does NOT satisfy new goal")
            rospy.loginfo(f"goal: {self.current_goal}, last in path: {self.current_path[-1]}")
            self.current_path.clear()
            self.current_path_cost = LARGE_COST
        elif self.current_path:
            rospy.logwarn("Keeping current path, satisfies new goal")

    def handle_new_edge_clear(self):
        self.edge_clear = self._new_edge_clear_msg.clear
        self.edge_clear_point = self._new_edge_clear_msg.trajectory_point

    def handle_new_controller_result(self):
        if self._new_controller_result_msg.status.status == 3:
            self.controller_active = False
        else:
            rospy.logerr(f"Unexpected result message status. message: \n{result_msg}")
            raise Exception('Throwing exception due to unexpected controller result')

    def start_planning_thread(self, reset_path=False, preempt=False, control_time_left=None):
        """ Create and start new planning thread

        Will tell the planner to return the first path solution it finds if there is not much time
        left before the controller finishes executing so we don't have to wait around for a plan
        while we don't have one to execute on. Otherwise, allow the planner to plan for however
        long is left before the controller completes executing so it can attempt to improve
        solutions via rewiring.
        """
        if reset_path:
            self.current_path.clear()
            self.current_path_cost = LARGE_COST
            with self.current_path_mutex:
                self._new_current_path = None
        if preempt:
            self.preempt_planner_pub.publish(Bool(True))
        if hasattr(self, 'planning_thread') and self.planning_thread.is_alive():
            rospy.loginfo("Planning thread still alive while attempting to re-start. joining...")
            self.planning_thread.join()

        return_first_solution = False if control_time_left else True
        self.planning_thread = Thread(target=self.robot_interface.plan_to_joint_goal,
                                      args=(self.current_goal, return_first_solution))
        if not return_first_solution:
            self.robot_interface.set_planning_time(control_time_left)
        self.planning_thread.start()
        rospy.loginfo(f"Started planning thread, return_first_solution: {return_first_solution}")

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
        rospy.loginfo("Initiating next move")
        self.current_path[1].time_from_start = rospy.Duration(self.control_dur)
        # generate and publish executing_to_state message for collision
        # checker
        joint_position_msg = JointTrajectoryPointStamped()
        joint_position_msg.trajectory_point = self.current_path[1]
        joint_position_msg.header.stamp = rospy.Time.now()
        self.executing_to_state_pub.publish(joint_position_msg)
        self.initiate_control()
        self.control_end_time = rospy.Time.now() + rospy.Duration(self.control_dur)
        # Set start state as state we are controlling to so that the
        # planner will generate plans starting from there
        self.current_state_positions = self.current_path[1].positions
        new_start_state = RobotState()
        new_start_state.joint_state.header.stamp = rospy.Time.now()
        new_start_state.joint_state.name = self.joint_names
        new_start_state.joint_state.position = self.current_state_positions
        joint_state = JointState()
        self.robot_interface.move_group.set_start_state(new_start_state)
        self.current_path.pop(0)
        self.current_path_cost = utils.path_cost(self.current_path)
        self.edge_clear = False

    @abstractmethod
    def initiate_control(self):
        pass

    def new_planner_goal(self, goal_msg):
        self.robot_interface.smooth_stop()
        self.current_goal = goal_msg.data
        plan_start_time = rospy.Time.now()
        self.robot_interface.go_to_joint_goal(self.current_goal, wait=False)
        plan_time = (rospy.Time.now() - plan_start_time).to_sec()
        rospy.logwarn(f"Time to plan: {plan_time}")

    def obstacles_changed(self, changed_msg):
        self.robot_interface.smooth_stop()
        plan_start_time = rospy.Time.now()
        self.robot_interface.go_to_joint_goal(self.current_goal, wait=False)
        plan_time = (rospy.Time.now() - plan_start_time).to_sec()
        rospy.logwarn(f"Time to plan: {plan_time}")
        with open(PLAN_TIME_FILE_PATH, 'w') as f:
            f.write(str(plan_time))


class RRTstarHwMonitor(RRTstarMonitor):

    def __init__(self):
        self.controller_topic = HW_CONTROLLER_TOPIC
        self.controller_msg_type = FollowJointTrajectoryAction
        self.controller_result_topic = HW_CONTROLLER_RESULT_TOPIC
        self.controller_result_msg_type = FollowJointTrajectoryActionResult
        super().__init__()
        rospy.loginfo("RRT* HARDWARE monitor initialized")

    def initiate_control(self):
        goal_point = self.current_path[1]
        trajectory_msg = FollowJointTrajectoryGoal()
        trajectory_msg.trajectory.joint_names = self.joint_names
        trajectory_msg.trajectory.points.append(goal_point)
        self.trajectory_client.send_goal(trajectory_msg)
        self.controller_active = True


class RRTstarSimMonitor(RRTstarMonitor):

    def __init__(self):
        self.controller_topic = SIM_CONTROLLER_TOPIC
        self.controller_msg_type = ExecuteTrajectoryAction
        self.controller_result_topic = SIM_CONTROLLER_RESULT_TOPIC
        self.controller_result_msg_type = ExecuteTrajectoryActionResult
        super().__init__()
        rospy.loginfo("RRT* SIMULATION monitor initialized")

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
    rospy.init_node("RRT_star_monitor")
    if rospy.get_param('/simulation', False):
        monitor = RRTstarSimMonitor()
    else:
        monitor = RRTstarHwMonitor()
    monitor.run()
