#!/usr/bin/env python3
"""

This node provides a planning & control interface to allow use of a
static planner (like RRTstar) in a dynamic environment.

Subscribes to the "new_planner_goal" and "obstacles_changed" topics for
dynamic information about the environment.


"""

import os

import actionlib
import rospy
import rospkg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from demo_interface import DemoInterface

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
PLAN_TIME_FILE_PATH = os.path.join(PLANNING_DATA_PATH, 'rrt_last_plan_time.txt')
CONTROLLER_TOPIC = "/position_joint_trajectory_controller/follow_joint_trajectory"
DESIRED_JOINT_STATE_TOPIC = "/joint_states_desired"
VELOCITY_MULTIPLIER = 0.2

class RRTstarMonitor():

    def __init__(self):
        self.d = DemoInterface(node_initialized=True)
        self.d.move_group.set_planner_id("RRTstarkConfigRealTimeTesting")
        self.d.set_planning_time(2.0)
        self.current_goal = []
        self.init_subscribers()
        self.trajectory_client = actionlib.SimpleActionClient(CONTROLLER_TOPIC,
                                                              FollowJointTrajectoryAction)
        while not self.trajectory_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo(f"Waiting for the {CONTROLLER_TOPIC} action server")

    def init_subscribers(self):
        self.new_goal_sub = rospy.Subscriber("/new_rrt_planner_goal", Float64MultiArray,
                                             self.new_planner_goal, queue_size=1)
        self.obstacles_changed_sub = rospy.Subscriber("/obstacles_changed", Bool,
                                                      self.obstacles_changed, queue_size=1)

    def new_planner_goal(self, goal_msg):
        self.d.smooth_stop()
        self.current_goal = goal_msg.data
        plan_start_time = rospy.Time.now()
        self.d.go_to_joint_goal(self.current_goal, wait=False)
        plan_time = (rospy.Time.now() - plan_start_time).to_sec()
        rospy.logwarn(f"Time to plan: {plan_time}")

    def obstacles_changed(self, changed_msg):
        self.d.smooth_stop()
        plan_start_time = rospy.Time.now()
        self.d.go_to_joint_goal(self.current_goal, wait=False)
        plan_time = (rospy.Time.now() - plan_start_time).to_sec()
        rospy.logwarn(f"Time to plan: {plan_time}")
        with open(PLAN_TIME_FILE_PATH, 'w') as f:
            f.write(str(plan_time))


if __name__ == "__main__":
    rospy.init_node("RRT_star_monitor")
    monitor = RRTstarMonitor()
    rospy.spin()
