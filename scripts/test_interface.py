#!/usr/bin/env python3

import json
import os
import numpy as np
import pandas as pd
import sys
import threading as th

import rospy
import rospkg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

from demo_interface import DemoInterface
from RRT_dynamic_interface import RRTPlannerControlInterface

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
SUMMARY_COLUMNS = ["RRTstar cost", "RTRRTstar cost", "Difference"]
DEFAULT_INITIAL_JOINT_GOAL = [-0.879063, -0.230783, -0.033678, -2.086562, 1.778457, 2.200953,
                              1.999634]
DEFAULT_FINAL_JOINT_GOAL = [-0.7849918264867694, 0.714459294555375, 2.081170739860527,
                            -2.13964710731112, -2.1925090777104494, 1.6207882359697081,
                            -0.6480528889305832]

class TestInterface():

    def __init__(self):
        self.d = DemoInterface(node_initialized=True)
        self.init_pubs()
        self.get_joint_goals()

    def get_joint_goals(self):
        self.initial_joint_goal = rospy.get_param("/test_interface/initial_joint_goal",
                                                  DEFAULT_INITIAL_JOINT_GOAL)
        self.final_joint_goal = rospy.get_param("/test_interface/final_joint_goal",
                                                DEFAULT_FINAL_JOINT_GOAL)

    def init_pubs(self):
        self.new_goal_pub = rospy.Publisher("/new_planner_goal", Float64MultiArray, queue_size=1)
        self.obstacles_changed_pub = rospy.Publisher("/obstacles_changed", Bool, queue_size=1)

    def setup(self, planner, start=[]):
        rospy.loginfo("Setting up")
        self.object_names = self.d.scene.get_known_object_names()
        for object_name in self.object_names:
            self.d.remove_object(object_name)
        self.d.set_planner_id("RRTstarkConfigDefault")
        if start:
            rospy.loginfo(f"Going to provided start: {start}")
            self.d.go_to_joint_goal(start)
        else:
            rospy.loginfo("Going to default start")
            self.d.go_to_start()
        self.d.set_planner_id(planner)

    def run_rtrrt_change_goal_test(self):
        self.setup("RTRRTstarkConfigDefault")
        final_joint_msg = Float64MultiArray()
        final_joint_msg.data = self.final_joint_goal
        rospy.loginfo("Sending initial joint goal")
        log_timer = th.Timer(2, rospy.loginfo, args=("Sending final joint goal",))
        change_goal_timer = th.Timer(2, self.new_rtrrt_goal_pub.publish, args=(final_joint_msg,))
        log_timer.start()
        change_goal_timer.start()
        self.d.plan_to_joint_goal(self.initial_joint_goal)

    def run_rrt_change_goal_test(self):
        (start_state, goal_state) = self.load_start_and_goal_states()
        rospy.loginfo(f"Loaded start: {start_state}")
        rospy.loginfo(f"Loaded goal: {goal_state}")
        self.setup("RRTstarkConfigDefault", start=start_state)
        joint_goal_msg = Float64MultiArray()
        joint_goal_msg.data = goal_state
        rospy.loginfo("Sending joint goal")
        self.d.go_to_joint_goal(goal_state)

    def load_start_and_goal_states(self):
        with open(os.path.join(PLANNING_DATA_PATH, 'RTRRTstar_run.json'), 'r') as f:
            RTRRT_json = json.load(f)
        num_goals = len(RTRRT_json['Goals'])
        start_state = RTRRT_json['Goal' + str(num_goals) + 'States'][0]
        last_goal_state = RTRRT_json['Goals'][-1]
        return (start_state, last_goal_state)

    def update_summary(self):
        (RRTstar_cost, RTRRTstar_cost, diff) = self.calculate_run_costs()
        run_db = pd.DataFrame([[RRTstar_cost, RTRRTstar_cost, diff]], columns=SUMMARY_COLUMNS)
        REPORT_PATH = os.path.join(PLANNING_DATA_PATH, 'summary.csv')
        if os.path.exists(REPORT_PATH):
            summary_db = pd.read_csv(REPORT_PATH)
        else:
            summary_db = pd.DataFrame(columns=SUMMARY_COLUMNS)
        summary_db = pd.concat([summary_db, run_db])
        summary_db.to_csv(REPORT_PATH, index=False)

    def calculate_run_costs(self):
        with open(os.path.join(PLANNING_DATA_PATH, 'RRTstar_run.json'), 'r') as f:
            RRT_json = json.load(f)
        with open(os.path.join(PLANNING_DATA_PATH, 'RTRRTstar_run.json'), 'r') as f:
            RTRRT_json = json.load(f)
        RRT_cost = self.calculate_cost(RRT_json['States'])
        # Extract RTRRT cost from second leg
        num_goals = len(RTRRT_json.items())-1
        last_goal_states_key = "Goal" + str(num_goals) + "States"
        RTRRT_states = RTRRT_json[last_goal_states_key]
        RTRRT_cost = self.calculate_cost(RTRRT_states)
        return (RRT_cost, RTRRT_cost, (RRT_cost - RTRRT_cost))

    def calculate_cost(self, states):
        cost = 0
        for i in range(len(states) - 1):
            s1 = states[i]
            s2 = states[i + 1]
            for j in range(len(states[i])):
                    diff = s1[j] - s2[j]
                    cost += diff ** 2
        return np.sqrt(cost)


if __name__ == "__main__":
    rospy.init_node("test_interface")
    ti = TestInterface()
    rospy.loginfo("Running RTRRT test")
    ti.run_rtrrt_change_goal_test()
    rospy.loginfo("Running RRT test")
    ti.run_rrt_change_goal_test()
    ti.update_summary()


