#!/usr/bin/env python3

import numpy as np

import rospy
from trajectory_msgs.msg import JointTrajectory


def log_cost(current_path_msg):
    path = current_path_msg.points
    cost = 0
    expected_next_cost = 0
    for i in range(len(path) - 1):
        position1 = np.array(path[i].positions)
        position2 = np.array(path[i+1].positions)
        cost += np.linalg.norm(position2 - position1, ord=1)
        if i == 0:
            continue
        expected_next_cost += np.linalg.norm(position2 - position1, ord=1)
    rospy.loginfo(f"COST: {cost}, EXPECTED_NEXT_COST: {expected_next_cost}")

if __name__ == "__main__":
    rospy.init_node("path_cost_logger")
    current_path_sub = rospy.Subscriber("/current_path", JointTrajectory, log_cost, queue_size=1)
    rospy.spin()
