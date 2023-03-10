#!/usr/bin/env python3

import os
import numpy as np
import pandas as pd

import rospy
import rospkg
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from robo_demo_msgs.msg import JointTrajectoryPointStamped
import utils

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
COST_DATA_FILE_PATH = os.path.join(PLANNING_DATA_PATH, 'cost_data.csv')


class PathRecorder():

    def __init__(self):
        self.current_goal = None
        self.initialize_db()
        self.initialize_path_traversed()
        self.init_subs()
        rospy.loginfo("Path Recorder initialized")

    def initialize_db(self):
        self.planning_process_str = rospy.get_param("/planning_process", "uninitialized")
        self.scenario_str = rospy.get_param("/scenario", "uninitialized")
        self.file_path = COST_DATA_FILE_PATH
        self.dict_db = {'Planning_Process': self.planning_process_str,
                        'Scenario': self.scenario_str,
                        'Total_Cost': 0,
                        'Step_Costs': []}
        # self.db = pd.load_csv()..

    def initialize_path_traversed(self):
        self.path_traversed = []
        current_state = rospy.wait_for_message("/joint_states", JointState)
        self.path_traversed.append(current_state.position[:7])

    def init_subs(self):
        self.new_goal_sub = rospy.Subscriber("/new_planner_goal", Float64MultiArray,
                                             self.new_goal_cb, queue_size=1)
        self.executing_to_state_sub = rospy.Subscriber("/executing_to_state",
                                                       JointTrajectoryPointStamped,
                                                       self.executing_to_state_cb, queue_size=1)

    def new_goal_cb(self, new_goal_msg):
        self.current_goal = new_goal_msg.data

    def executing_to_state_cb(self, new_execution_msg):
        new_positions = new_execution_msg.trajectory_point.positions
        self.path_traversed.append(new_positions)
        if utils.almost_equal(self.current_goal, new_positions):
            self.compute_path_info()
            self.store_path_info()
            self.path_traversed.clear()

    def compute_path_info(self):
        total_cost, step_costs = utils.path_cost(self.path_traversed, steps=True)
        self.dict_db['Total_Cost'] = total_cost
        self.dict_db['Step_Costs'] = step_costs

    def store_path_info(self):
        rospy.loginfo(f"(not actually but will) Saving path info to...{self.file_path}")
        rospy.loginfo(f"dict_db: {self.dict_db}")
        rospy.loginfo("-----------------------------------------")


if __name__ == "__main__":
    rospy.init_node("path_recorder")
    recorder = PathRecorder()
    rospy.spin()
