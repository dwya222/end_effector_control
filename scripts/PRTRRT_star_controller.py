#!/usr/bin/env python3
"""
Parallelized Real-Time RRT* Controller
"""
from abc import abstractmethod
from enum import Enum
from threading import RLock

import rospy
import actionlib
from std_msgs.msg import Float64MultiArray
from robo_demo_msgs.msg import BoolStamped
from moveit_msgs.msg import (ExecuteTrajectoryAction, ExecuteTrajectoryGoal,
                             ExecuteTrajectoryActionResult)
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal,
                              FollowJointTrajectoryActionResult)
from trajectory_msgs.msg import JointTrajectory
from robo_demo_msgs.msg import JointTrajectoryPointStamped
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
        self.control_time = rospy.get_param('/control_time', 1.0)
        self.controller_active = False
        self.mutex = RLock()
        self.edge_clear_time = rospy.Time.now()
        # DEBUG
        self.current_path_cb_count = 0
        self.edge_clear_cb_count = 0
        self.controller_result_cb_count = 0
        self.executing_to_state_pub_count = 0

    def setup_controller(self):
        self.trajectory_client = actionlib.SimpleActionClient(self.controller_topic,
                                                              self.controller_msg_type)
        while not self.trajectory_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo_throttle(1.0, f"Waiting for the {SIM_CONTROLLER_TOPIC} action server")
        rospy.loginfo(f"Controller connected. Topic: {SIM_CONTROLLER_TOPIC}")

    def init_subs_pubs(self):
        self.current_path_sub = rospy.Subscriber("/current_path", JointTrajectory,
                                                 self.current_path_cb, queue_size=1)
        self.edge_clear_sub = rospy.Subscriber("/edge_clear", BoolStamped,
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
        with self.mutex:
            # DEBUG
            self.current_path_cb_count += 1
            rospy.logwarn(f"current_path callback {self.current_path_cb_count}")
            # rospy.loginfo("PRTRRT controller recieved new current path")
            self.current_path = current_path_msg.points
            # The edge clear msg could have already come in for the next
            # step in this path, so call keep self.edge_clear the same
            # if that happened
            if self.edge_clear_time < current_path_msg.header.stamp:
                self.edge_clear = False
            else:
                rospy.logwarn(f"OUT OF ORDER CB DETECTED edge_clear time {self.edge_clear_time} "
                              f"current_path time: {current_path_msg.header.stamp}")
            rospy.loginfo(f"current_path_cb edge_clear: {self.edge_clear}, {current_path_msg.header.stamp}")

    def edge_clear_cb(self, clear_msg):
        with self.mutex:
            # DEBUG
            self.edge_clear_cb_count += 1
            rospy.logwarn(f"edge_clear callback {self.edge_clear_cb_count}")
            # rospy.loginfo(f"PRTRRT controller: Edge clear cb. data: {clear_msg.data}")
            self.edge_clear = clear_msg.data
            self.edge_clear_time = clear_msg.header.stamp
            rospy.loginfo(f"edge_clear_cb edge_clear: {self.edge_clear}, {self.edge_clear_time}")
            if self.edge_clear and not self.controller_active:
                self.initiate_next_move()

    def new_goal_cb(self, new_goal_msg):
        with self.mutex:
            if self.current_path:
                self.current_path.clear()

    def controller_result_cb(self, result_msg):
        with self.mutex:
            # DEBUG
            self.controller_result_cb_count += 1
            rospy.logwarn(f"controller_result callback {self.controller_result_cb_count}")
            if result_msg.status.status == 3:
                # rospy.loginfo(f"PRTRRT controller successfully reached next state, edge_clear? {self.edge_clear}")
                self.controller_active = False
                if len(self.current_path) == 1:
                    # rospy.loginfo("Goal Achieved: reached end of current_path")
                    return
                if self.edge_clear:
                    self.initiate_next_move()
            else:
                rospy.logerr(f"Unexpected result message status. message: \n{result_msg}")

    def initiate_next_move(self):
        if not self.current_path:
            rospy.logwarn("Attempted to initiate next move without current path, returning")
            return
        # DEBUG
        self.executing_to_state_pub_count += 1
        rospy.logwarn(f"executing to state pub {self.executing_to_state_pub_count}")
        joint_position_msg = JointTrajectoryPointStamped()
        self.current_path[1].time_from_start = rospy.Duration(self.control_time)
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
        # rospy.loginfo("Initiating control to next state in current_path")
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
        # rospy.loginfo("Initiating control to next state in current_path")
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
