#!/usr/bin/env python3

import os
import numpy as np
import pandas as pd

import rospy
import rospkg
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState

from robo_demo_msgs.msg import JointTrajectoryPointStamped
import utils

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
COST_DATA_FILE_PATH = os.path.join(PLANNING_DATA_PATH, 'planning_process_cost_data.csv')
DATABASE_COLUMNS = ["Scenario", "Planning Process", "Total Cost", "Step Costs"]


class PathRecorder():

    def __init__(self):
        self.current_goal = None
        self.initialize_db()
        self.initialize_path_traversed()
        self.init_subs()
        rospy.loginfo("Path Recorder initialized")

    def initialize_db(self):
        self.db_path = COST_DATA_FILE_PATH
        self.db_columns = DATABASE_COLUMNS
        if os.path.exists(self.db_path):
            self.db = pd.read_csv(self.db_path)
            rospy.loginfo(f"initialized saved db: \n{self.db}")
        else:
            self.db = pd.DataFrame(columns=self.db_columns)
            rospy.loginfo(f"initialized new db: \n{self.db}")

    def initialize_path_traversed(self):
        self.path_traversed = []
        current_state = rospy.wait_for_message("/joint_states", JointState)
        self.path_traversed.append(current_state.position[:7])

    def init_subs(self):
        self.new_goal_sub = rospy.Subscriber("/new_planner_goal", Float64MultiArray,
                                             self.new_goal_cb, queue_size=1)
        self.planning_process_sub = rospy.Subscriber("/planning_process", String,
                                                     self.planning_process_cb, queue_size=1)
        self.executing_to_state_sub = rospy.Subscriber("/executing_to_state",
                                                       JointTrajectoryPointStamped,
                                                       self.executing_to_state_cb, queue_size=1)

    def new_goal_cb(self, new_goal_msg):
        self.current_goal = new_goal_msg.data

    def planning_process_cb(self, planning_process_msg):
        self.planning_process_str = planning_process_msg.data

    def executing_to_state_cb(self, new_execution_msg):
        new_positions = new_execution_msg.trajectory_point.positions
        self.path_traversed.append(new_positions)
        if utils.almost_equal(self.current_goal, new_positions):
            self.store_path_info()
            self.path_traversed.clear()

    def store_path_info(self):
        scenario = rospy.get_param("/scenario", "uninitialized")
        planning_process = rospy.get_param("/planning_process", "uninitialized")
        total_cost, step_costs = utils.path_cost(self.path_traversed, steps=True)
        db_entry = pd.DataFrame([[scenario, planning_process, total_cost, step_costs]],
                                columns=self.db_columns)
        rospy.loginfo(f"Saving path info to...{self.db_path}")
        rospy.loginfo(f"db_entry:\n{db_entry}")
        rospy.loginfo("-----------------------------------------")
        self.db = pd.concat([self.db, db_entry])
        self.db.to_csv(self.db_path, index=False)


if __name__ == "__main__":
    rospy.init_node("path_recorder")
    recorder = PathRecorder()
    rospy.spin()
