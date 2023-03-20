#!/usr/bin/env python3

import json
import os
import numpy as np
import pandas as pd
import sys
import threading as th

import rospy
import actionlib
import rospkg
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, String

from demo_interface import DemoInterface
from robo_demo_msgs.srv import RunPlanningTest, RunPlanningTestResponse

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
SUMMARY_COLUMNS = ["Test", "RRTstar cost", "RTRRTstar cost", "Difference"]
RRT_SUMMARY_COLUMNS = ["Test", "RRTstar shortest path cost", "RRTstar cost", "RRTstar plan time"]
SERVICE_FEEDBACK_TOPIC = "/move_group/feedback"
START_STATE = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
INITIAL_JOINT_GOAL = [0.9022525451956217, -1.0005812062660042, -1.7602947518592436,
                      -2.7099525294963933, -0.1420045228964755, 3.493668294307763,
                      -0.3472854722693375]
INITIAL_JOINT_GOAL2 = [-0.5578778636322044, 0.908623569993187, -0.38844591131009487,
                       -1.2094721531006272, 0.3444179383032919, 2.0541426810783356,
                       -0.18421686175609792]
FINAL_JOINT_GOAL = [1.1901980109241104, 0.9615559746057705, -0.5881359185350531,
                    -1.2015471132200233, 0.5281574185640393, 2.0160775068768824,
                    1.3658315499054479]
# (x, y, z, r)
(OBS_X, OBS_Y, OBS_Z, OBS_R) = (0.4, -0.4, 0.4, 0.05)
OBS_IN_BEST_PATH = [(0.4, -0.4, 0.4, 0.05)]
OBS_OUT_OF_WAY = [(1.4, -1.5, 1.2, 0.05), (1.0, -0.7, 0.2, 0.07), (-1.0, -0.7, 0.2, 0.1),
                  (-1.0, 0.7, 0.2, 0.05), (0.2, 0.7, 0.6, 0.08)]
OBS_LIST = OBS_IN_BEST_PATH + OBS_OUT_OF_WAY

class TestInterface():

    def __init__(self):
        self.init_robot_interface()
        self.init_params()
        self.new_goal_pub = rospy.Publisher("/new_planner_goal", Float64MultiArray, queue_size=1)
        self.test_service = rospy.Service("test_interface_service", RunPlanningTest, self.run_test)
        # self.test_service = rospy.ServiceProxy('test_interface_service', RunPlanningTest)
        rospy.sleep(0.5) # allow pub to connect
        rospy.loginfo("TestInterface initialized")

    def init_robot_interface(self):
        self.robot_interface = DemoInterface(node_initialized=True)
        self.robot_interface.set_planner_id("RRTstarkConfigDefault")
        self.robot_interface.set_planning_time(2.0)

    def init_params(self):
        self.default_start_state = rospy.get_param("/test_interface/start_state", START_STATE)
        self.initial_joint_goal = rospy.get_param("/test_interface/initial_joint_goal",
                                                  INITIAL_JOINT_GOAL)
        self.final_joint_goal = rospy.get_param("/test_interface/final_joint_goal",
                                                FINAL_JOINT_GOAL)
        self.layout = MultiArrayLayout()

    def run_test(self, test_msg):
        test = test_msg.scenario
        rospy.loginfo(f"Attempting to run '{test}' test")
        success=False
        if test == "change_goal":
            rospy.set_param("/scenario", "Change Goal")
            dyn_time = rospy.get_param("/test_interface/dyn_time", 1.75)
            self.run_change_goal_test(dyn_time)
            success=True
        elif test == "add_obstacle":
            rospy.set_param("/scenario", "Add Obstacle")
            dyn_time = rospy.get_param("/test_interface/dyn_time", 1.0)
            self.run_add_obstacle_test(dyn_time)
            success=True
        elif test == "add_obstacle_change_goal":
            rospy.set_param("/scenario", "Add Obstacle and Change Goal")
            dyn_time = rospy.get_param("/test_interface/dyn_time", 2.0)
            self.initial_joint_goal = rospy.get_param("/test_interface/initial_joint_goal",
                                                      INITIAL_JOINT_GOAL2)
            self.run_add_obstacle_change_goal_test(dyn_time)
            success=True
        else:
            rospy.logerr(f"Invalid test name: '{test}'")
        return RunPlanningTestResponse(success=success)

    def setup(self, start=[]):
        start = self.default_start_state if start==[] else start
        rospy.loginfo("Setting up")
        # self.robot_interface.remove_object("obstacle")
        self.object_names = self.robot_interface.scene.get_known_object_names()
        for object_name in self.object_names:
            self.robot_interface.remove_object(object_name)
        rospy.loginfo(f"Moving to start: {start}")
        self.robot_interface.go_to_joint_goal(start)
        rospy.sleep(1)

    def run_change_goal_test(self, dyn_time):
        self.setup()
        rospy.loginfo("Running change_goal test")
        rospy.loginfo("Sending initial joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.initial_joint_goal))
        rospy.sleep(dyn_time)
        rospy.loginfo("Sending final joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.final_joint_goal))

    def run_add_obstacle_test(self, dyn_time):
        self.setup()
        rospy.loginfo("Running add_obstacle test")
        rospy.loginfo("Sending initial joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.initial_joint_goal))
        rospy.sleep(dyn_time)
        rospy.loginfo("Adding obstacle")
        (x, y, z, r) = (OBS_X, OBS_Y, OBS_Z, OBS_R)
        self.robot_interface.publish_object_xyz("obstacle", x, y, z, r, 'sphere')

    def run_add_obstacle_change_goal_test(self, dyn_time):
        self.setup()
        rospy.loginfo("Running add_obstacle_change_goal test")
        rospy.loginfo("Sending initial joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.initial_joint_goal))
        rospy.sleep(dyn_time)
        rospy.loginfo("Adding obstacle and then sending new goal")
        (x, y, z, size) = (0.8, 0.0, 0.4, (0.5, 0.02, 0.4))
        self.robot_interface.publish_object_xyz("obstacle", x, y, z, size)
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.final_joint_goal))


if __name__ == "__main__":
    rospy.init_node("test_interface")
    ti = TestInterface()
    rospy.spin()
    # test = rospy.get_param('/test_interface/test', 'change_goal')
    # rospy.logwarn(f"TestInterface running test: {test}")
    # ti.run_test(test)
