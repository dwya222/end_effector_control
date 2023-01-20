#!/usr/bin/env python3
"""
Parallelized Real-Time RRT* Controller
"""
from abc import abstractmethod
from enum import Enum

import rospy
import actionlib
from std_msgs.msg import Bool
from moveit_msgs.msg import (ExecuteTrajectoryAction, ExecuteTrajectoryGoal,
                             ExecuteTrajectoryActionResult)
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal,
                              FollowJointTrajectoryActionResult)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from demo_interface import DemoInterface

HW_CONTROLLER_TOPIC = "/position_joint_trajectory_controller/follow_joint_trajectory"
HW_CONTROLLER_RESULT_TOPIC = HW_CONTROLLER_TOPIC + "/result"
SIM_CONTROLLER_TOPIC = "/execute_trajectory"
SIM_CONTROLLER_RESULT_TOPIC = SIM_CONTROLLER_TOPIC + "/result"
PANDA_JOINT_NAMES = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                     'panda_joint5', 'panda_joint6', 'panda_joint7']
DESIRED_JOINT_STATE_TOPIC = "/joint_states_desired"
VELOCITY_MULTIPLIER = 0.2


class PRTRRTstarController():

    def __init__(self):
        self.setup_controller()
        self.init_subs_pubs()
        self.joint_names = PANDA_JOINT_NAMES
        self.control_execution_time = rospy.get_param('/control_execution_time', 1.0)
        self.controller_active = False

    def setup_controller(self):
        self.trajectory_client = actionlib.SimpleActionClient(self.controller_topic,
                                                              self.controller_msg_type)
        while not self.trajectory_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo(f"Waiting for the {SIM_CONTROLLER_TOPIC} action server")
        rospy.loginfo(f"Controller connected. Topic: {SIM_CONTROLLER_TOPIC}")

    def init_subs_pubs(self):
        self.current_path_sub = rospy.Subscriber("/current_path", JointTrajectory,
                                                 self.current_path_cb, queue_size=1)
        self.edge_clear_sub = rospy.Subscriber("/edge_clear", Bool,
                                               self.edge_clear_cb, queue_size=1)
        self.controller_result_sub = rospy.Subscriber(self.controller_result_topic,
                                                      self.controller_result_msg_type,
                                                      self.controller_result_cb, queue_size=1)
        self.executing_to_state_pub = rospy.Publisher("/executing_to_state", JointTrajectoryPoint,
                                                      queue_size=1)
        rospy.loginfo(f"Publisher and subscribers initialized")

    def current_path_cb(self, current_path_msg):
        rospy.loginfo("PRTRRT controller recieved new current path")
        self.current_path = current_path_msg.points
        rospy.loginfo(f"Initial current path length: {len(self.current_path)}")
        self.edge_clear = False

    def edge_clear_cb(self, clear_msg):
        rospy.loginfo("PRTRRT controller: Edge clear cb")
        self.edge_clear = clear_msg.data
        if self.edge_clear and not self.controller_active:
            self.initiate_next_move()

    def controller_result_cb(self, result_msg):
        if result_msg.status.status == 3:
            rospy.loginfo("PRTRRT controller successfully reached next state")
            self.controller_active = False
            rospy.loginfo(f"current path length: {len(self.current_path)}")
            if len(self.current_path) == 1:
                rospy.loginfo("Goal Achieved: reached end of current_path")
                return
            if self.edge_clear:
                self.initiate_next_move()
        else:
            rospy.logerr(f"Unexpected result message status. message: \n{result_msg}")

    def initiate_next_move(self):
        self.current_path[1].time_from_start = rospy.Duration(self.control_execution_time)
        self.executing_to_state_pub.publish(self.current_path[1])
        self.initiate_control()
        self.current_path.pop(0)
        self.edge_clear = False

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
        rospy.loginfo("Initiating control to next state in current_path")
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
        rospy.loginfo("Initiating control to next state in current_path")
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
        monitor = PRTRRTstarSimController()
    else:
        monitor = PRTRRTstarHwController()
    rospy.spin()
