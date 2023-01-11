#!/usr/bin/env python3
"""

This node provides a planning & control interface to allow use of a
static planner (like RRTstar) in a dynamic environment.

It subscribes to the "new_planner_goal" and "obstacles_changed" topics
for dynamic information about the environment.

TODO:
    - create stopping method that works smoothly on hardware


"""

import os

import rospy
import rospkg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

from demo_interface import DemoInterface

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
PLAN_TIME_FILE_PATH = os.path.join(PLANNING_DATA_PATH, 'rrt_last_plan_time.txt')

class RRTPlannerControlInterface():

    def __init__(self):
        self.d = DemoInterface(node_initialized=True)
        self.d.move_group.set_planner_id("RRTstarkConfigRealTimeTesting")
        self.d.set_planning_time(2.0)
        self.current_goal = []
        self.init_subscribers()

    def init_subscribers(self):
        self.new_goal_sub = rospy.Subscriber("/new_rrt_planner_goal", Float64MultiArray,
                                             self.new_planner_goal, queue_size=1)
        self.obstacles_changed_sub = rospy.Subscriber("/obstacles_changed", Bool,
                                                      self.obstacles_changed, queue_size=1)

    def new_planner_goal(self, goal_msg):
        self.d.move_group.stop()
        self.current_goal = goal_msg.data
        plan_start_time = rospy.Time.now()
        self.d.go_to_joint_goal(self.current_goal, wait=False)
        plan_time = (rospy.Time.now() - plan_start_time).to_sec()
        rospy.logwarn(f"Time to plan: {plan_time}")

    def obstacles_changed(self, changed_msg):
        self.d.move_group.stop()
        if self.current_goal:
            plan_start_time = rospy.Time.now()
            self.d.go_to_joint_goal(self.current_goal, wait=False)
            plan_time = (rospy.Time.now() - plan_start_time).to_sec()
            rospy.logwarn(f"Time to plan: {plan_time}")
            with open(PLAN_TIME_FILE_PATH, 'w') as f:
                f.write(str(plan_time))

if __name__ == "__main__":
    rospy.init_node("RRT_dynamic_interface")
    pci = RRTPlannerControlInterface()
    rospy.spin()
