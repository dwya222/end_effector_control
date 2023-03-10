#!/usr/bin/env python3

import os
import pandas as pd

import rospy
import rospkg
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

ITERATIONS = 100
# Database setup
rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
DATA_PATH = os.path.join(PLANNING_DATA_PATH, 'maintenance_data.csv')


if __name__ == "__main__":
    rospy.init_node("automate_testing_node")
    test_srv = rospy.ServiceProxy('/change_goal_test', Trigger)
    iterations = ITERATIONS
    if os.path.exists(DATA_PATH):
        rospy.loginfo("maintenance data found, subtracting iterations")
        df = pd.read_csv(DATA_PATH)
        count = len(df)
        iterations -= count
    else:
        rospy.loginfo("No saved maintenance data found")
    rospy.loginfo(f"Running change goal test for {iterations} iterations")
    rospy.sleep(0.5)
    for i in range(iterations):
        rospy.loginfo(f"Running iteration {i+1}/{iterations}")
        response = test_srv()
        rospy.loginfo(f"Received response: {response}")
