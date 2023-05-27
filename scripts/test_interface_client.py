#!/usr/bin/env python3

import rospy

from robo_demo_msgs.srv import RunPlanningTest


if __name__ == "__main__":
    rospy.init_node("test_interface_client")
    rospy.wait_for_service("/test_interface_service")
    planning_test_srv = rospy.ServiceProxy('/test_interface_service', RunPlanningTest)
    test_type = "add_obstacle_change_goal"
    rospy.loginfo(f"Attempting to run {test_type} planning test")
    response = planning_test_srv(test_type)
    rospy.loginfo(f"Received response: {response}")
