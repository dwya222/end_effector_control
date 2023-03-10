#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

from demo_interface import DemoInterface

if __name__ == "__main__":
    rospy.init_node("planner_initializer")
    robot_interface = DemoInterface(node_initialized=True)
    robot_interface.set_planner_id("RTRRTstarkConfigDefault")
    while not rospy.is_shutdown():
        new_goal = rospy.wait_for_message("/new_planner_goal", Float64MultiArray)
        robot_interface.plan_to_joint_goal(new_goal.data)
