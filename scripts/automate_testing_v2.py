#!/usr/bin/env python3

import pandas as pd
import os

import rospy
import rospkg
from std_msgs.msg import Bool

from robo_demo_msgs.srv import RunPlanningTest

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
COMMON_DATA_PATH = os.path.join(PLANNING_DATA_PATH, 'planning_data.csv')

DEFAULT_TEST_DICT = {'change_goal': 10, 'add_obstacle': 10, 'add_obstacle_change_goal': 10}

class TestAutomator():

    def __init__(self, test_dict=DEFAULT_TEST_DICT):
        self.test_dict = test_dict
        self.update_test_counts()
        self.planning_test_srv = rospy.ServiceProxy('/test_interface_service', RunPlanningTest)
        rospy.sleep(0.5)
        rospy.loginfo("TestAutomator initialized, running tests")
        self.run_tests()

    def update_test_counts(self):
        if os.path.exists(COMMON_DATA_PATH):
            df = pd.read_csv(COMMON_DATA_PATH)
            (t1, t2, t3) = ("change_goal", "add_obstacle", "add_obstacle_change_goal")
            t1_count = len(df[df["Scenario"]==t1])
            t2_count = len(df[df["Scenario"]==t2])
            t3_count = len(df[df["Scenario"]==t3])
            self.test_dict[t1] -= t1_count
            self.test_dict[t2] -= t2_count
            self.test_dict[t3] -= t3_count
            rospy.loginfo(f"counts: {t1_count}, {t2_count}, {t3_count}")

    def run_tests(self):
        for (test_type, iterations) in self.test_dict.items():
            for i in range(iterations):
                rospy.loginfo(f"Running {test_type}, iteration: {i+1}/{iterations}")
                response = self.planning_test_srv(test_type)
                rospy.loginfo(f"Received response: {response}")


if __name__ == "__main__":
    rospy.init_node("automate_testing_node")
    test_dict = {'change_goal': 50, 'add_obstacle': 50, 'add_obstacle_change_goal': 50}
    ta = TestAutomator(test_dict)
    # ta = TestAutomator()
