#!/usr/bin/env python3
"""

This node provides a planning & control interface to allow use of a
static planner (like RRTstar) in a dynamic environment.

It subscribes to the "new_planner_goal" and "obstacles_changed" topics
for dynamic information about the environment.

TODO:
    - create stopping method that works smoothly on hardware
    - create custom config rather than overwritting RRTstarkConfigDefault
    - figure out a way to create a better planner termination condition that is more comparative to
      RTRRT* (i.e. terminate planning when a path is first solved for rather than waiting 't'
      seconds)


"""

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

from demo_interface import DemoInterface

class RRTPlannerControlInterface():

    def __init__(self):
        self.d = DemoInterface(node_initialized=True)
        self.d.move_group.set_planner_id("RRTstarkConfigDefault")
        self.d.set_planning_time(0.5)
        self.init_subscribers()

    def init_subscribers(self):
        self.new_goal_sub = rospy.Subscriber("/new_rrt_planner_goal", Float64MultiArray,
                                             self.new_planner_goal, queue_size=1)
        self.obstacles_changed_sub = rospy.Subscriber("/obstacles_changed", Bool,
                                                      self.obstacles_changed, queue_size=1)

    def new_planner_goal(self, goal_msg):
        self.d.move_group.stop()
        self.current_goal = goal_msg.data
        self.d.go_to_joint_goal(self.current_goal, wait=False)

    def obstacles_changed(self, changed_msg):
        self.d.move_group.stop()
        self.d.go_to_joint_goal(self.current_goal, wait=False)

if __name__ == "__main__":
    rospy.init_node("RRT_dynamic_interface")
    pci = RRTPlannerControlInterface()
    rospy.spin()
