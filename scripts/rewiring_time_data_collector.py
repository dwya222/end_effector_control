#!/usr/bin/env python3

import os
import pandas as pd

import rospy
import rospkg
from std_msgs.msg import Float64, Float64MultiArray

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
DATABASE_FILE_PATH = os.path.join(PLANNING_DATA_PATH, 'rewiring_time_data.csv')
DATABASE_COLUMNS = ["Goal Number", "Rewire Action Number", "Rewire Time Per Node"]


class RewireTimeCollector():

    def __init__(self):
        self.rewire_db = pd.DataFrame(columns=DATABASE_COLUMNS)
        self.goal_count = 0
        self.new_goal = False
        self.rewiring_time_sub = rospy.Subscriber("/rewire_time", Float64, self.rewire_time_cb,
                                                  queue_size=10)
        self.new_goal_sub = rospy.Subscriber("/new_planner_goal", Float64MultiArray,
                                             self.new_goal_cb, queue_size=1)

    def rewire_time_cb(self, rewire_time_msg):
        if self.new_goal:
            self.goal_count += 1
            self.rewire_action_count = 0
            self.new_goal = False
        self.rewire_action_count += 1
        entry = pd.DataFrame([[self.goal_count, self.rewire_action_count, rewire_time_msg.data]],
                             columns=DATABASE_COLUMNS)
        self.rewire_db = pd.concat([self.rewire_db, entry])
        self.rewire_db.to_csv(DATABASE_FILE_PATH, index=False)

    def new_goal_cb(self, new_goal_msg):
        self.new_goal = True


if __name__ == "__main__":
    rospy.init_node("rewiring_time_data_collector")
    rtc = RewireTimeCollector()
    rospy.spin()
