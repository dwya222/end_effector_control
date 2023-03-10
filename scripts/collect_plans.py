#!/usr/bin/env python3

import os
import pandas as pd

import rospy
import rospkg
from trajectory_msgs.msg import JointTrajectory

from demo_interface import DemoInterface
import utils

# Database setup
rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
DATA_PATH = os.path.join(PLANNING_DATA_PATH, 'maintenance_data_from_scratch.csv')
COLUMNS = ["Cost"]
# Joint state setup
START_TARGET = [0.9022525451956217, -1.0005812062660042, -1.7602947518592436, -2.7099525294963933,
                 -0.1420045228964755, 3.493668294307763, -0.3472854722693375]
FINAL_TARGET = [1.1901980109241104, 0.9615559746057705, -0.5881359185350531, -1.2015471132200233,
               0.5281574185640393, 2.0160775068768824, 1.3658315499054479]

ITERATIONS = 50


class PlanCollector():

    def __init__(self):
        self.init_db()
        self.robot_interface = DemoInterface(node_initialized=True)
        self.robot_interface.set_planner_id("RRTstarkConfigRealTimeTesting")
        self.costs = []
        self.collect = False
        self.current_path_sub = rospy.Subscriber('/current_path', JointTrajectory,
                                                 self.current_path_cb)
        self.robot_interface.go_to_joint_goal(START_TARGET)

    def init_db(self):
        self.db_path = DATA_PATH
        self.db_columns = COLUMNS
        if os.path.exists(self.db_path):
            self.db = pd.read_csv(self.db_path)
            rospy.loginfo(f"Initialized saved db: \n{self.db}")
        else:
            self.db = pd.DataFrame(columns=self.db_columns)
            rospy.loginfo(f"Initialized new common db: \n{self.db}")

    def current_path_cb(self, path):
        cost = utils.path_cost(path.points)
        db_entry = pd.DataFrame([[cost]], columns=self.db_columns)
        self.db = pd.concat([self.db, db_entry])
        self.db.to_csv(self.db_path, index=False)
        rospy.loginfo(f"Cost of plan: {cost}")
        if self.collect:
            self.costs.append(cost)
            rospy.loginfo(f"Cost added, costs:\n{self.costs}")

    def get_plans(self, iterations):
        self.collect = True
        for i in range(iterations):
            rospy.loginfo(f"Planning for iteration {i+1}/{iterations}")
            self.robot_interface.plan_to_joint_goal(FINAL_TARGET, return_first_solution=True)



if __name__ == "__main__":
    rospy.init_node("plan_collector")
    pc = PlanCollector()
    pc.get_plans(ITERATIONS)
    rospy.spin()
