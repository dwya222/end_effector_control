#!/usr/bin/env python3

import json
import os
import numpy as np
import pandas as pd
import sys
import threading as th

import rospy
from rospy.exceptions import ROSException
import actionlib
import rospkg
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, String, Bool

from demo_interface import DemoInterface
from robo_demo_msgs.srv import RunPlanningTest, RunPlanningTestResponse
import utils

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
SUMMARY_COLUMNS = ["Test", "RRTstar cost", "RTRRTstar cost", "Difference"]
RRT_SUMMARY_COLUMNS = ["Test", "RRTstar shortest path cost", "RRTstar cost", "RRTstar plan time"]
SERVICE_FEEDBACK_TOPIC = "/move_group/feedback"
# For change goal test and add obstacle change goal test
START_STATE1 = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
INITIAL_GOAL1 = [0.9022525451956217, -1.0005812062660042, -1.7602947518592436, -2.7099525294963933,
                 -0.1420045228964755, 3.493668294307763, -0.3472854722693375]
FINAL_GOAL1 = [1.1901980109241104, 0.9615559746057705, -0.5881359185350531, -1.2015471132200233,
               0.5281574185640393, 2.0160775068768824, 1.3658315499054479]
# For add obs test
START_STATE2 = [1.29166488362329, -1.3326397337658384, -0.3771016570403198, -2.289747772176397,
                -1.6521502131549706, 2.080651449852423, -1.4296012169487267]
INITIAL_GOAL2 = [1.9414542111175392, -1.083334097524826, -1.5890062480581033, -1.8016281184439042,
                -2.857636979076863, 2.562539121147276, -0.559502583358445]
# For add obs change goal test
INITIAL_GOAL3 = [-0.5578778636322044, 0.908623569993187, -0.38844591131009487, -1.2094721531006272,
                 0.3444179383032919, 2.0541426810783356, -0.18421686175609792]

# (x, y, z, r)
(OBS_X, OBS_Y, OBS_Z, OBS_R) = (0.6, 0.0, 0.6, 0.03)
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
        self.start_state_dict = {'change_goal': START_STATE1,
                                 'add_obstacle': START_STATE2,
                                 'add_obstacle_change_goal': START_STATE1}
        self.goal_state_dict = {'change_goal': (INITIAL_GOAL1, FINAL_GOAL1),
                                'add_obstacle': INITIAL_GOAL2,
                                'add_obstacle_change_goal': (INITIAL_GOAL3, FINAL_GOAL1)}
        self.layout = MultiArrayLayout()
        self.priming_secs = rospy.get_param('/planner_priming_secs', 0.0)
        rospy.loginfo(f"priming secs: {self.priming_secs}")

    def run_test(self, test_msg):
        rospy.loginfo(f"Attempting to run '{test}' test")
        rospy.set_param("/scenario", test_msg.scenario)
        if test_msg.scenario == "change_goal":
            dyn_time = rospy.get_param("/test_interface/dyn_time", 1.75)
            self.run_change_goal_test(dyn_time + self.priming_secs)
        elif test == "add_obstacle":
            dyn_time = rospy.get_param("/test_interface/dyn_time", 0.75)
            self.run_add_obstacle_test(dyn_time + self.priming_secs)
        elif test == "add_obstacle_change_goal":
            dyn_time = rospy.get_param("/test_interface/dyn_time", 2.0)
            self.run_add_obstacle_change_goal_test(dyn_time + self.priming_secs)
        else:
            rospy.logerr(f"Invalid test name: '{test}'")
            return RunPlanningTestResponse(success=success)
        rospy.loginfo("Waiting for goal achieved")
        try:
            rospy.wait_for_message("/goal_achieved", Bool, timeout=15)
            return RunPlanningTestResponse(success=True)
        except ROSException as e:
            return RunPlanningTestResponse(success=False, message=f"Encountered exception: {e}")

    def setup(self, start):
        rospy.loginfo("Setting up")
        self.initializing_test_pub.publish(Bool(True))
        self.robot_interface.remove_object("obstacle")
        # self.object_names = self.robot_interface.scene.get_known_object_names()
        # for object_name in self.object_names:
        #     self.robot_interface.remove_object(object_name)
        rospy.loginfo(f"Setting to start: {start}")
        current_state = self.robot_interface.move_group.get_current_joint_values()
        if utils.almost_equal(current_state, start):
            rospy.loginfo("Already at start")
        else:
            rospy.loginfo("Publishing start as goal and waiting for goal_achieved")
            self.new_goal_pub.publish(Float64MultiArray(self.layout, start))
            goal_achieved_msg = rospy.wait_for_message("/goal_achieved", Bool)
            rospy.loginfo(f"Start achieved: {goal_achieved_msg.data}")
        self.initializing_test_pub.publish(Bool(True))

    def run_change_goal_test(self, dyn_time):
        self.setup(self.start_state_dict['change_goal'])
        rospy.loginfo("Running change_goal test")
        rospy.loginfo("Sending initial joint goal")
        (initial_goal, final_goal) = self.goal_state_dict['change_goal']
        self.new_goal_pub.publish(Float64MultiArray(self.layout, initial_goal))
        rospy.sleep(dyn_time)
        rospy.loginfo("Sending final joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, final_goal))

    def run_add_obstacle_test(self, dyn_time):
        self.setup(self.start_state_dict['add_obstacle'])
        rospy.loginfo("Running add_obstacle test")
        rospy.loginfo("Sending initial joint goal")
        initial_goal = self.goal_state_dict['add_obstacle']
        self.new_goal_pub.publish(Float64MultiArray(self.layout, initial_goal))
        rospy.sleep(dyn_time)
        rospy.loginfo("Adding obstacle")
        (x, y, z, r) = (OBS_X, OBS_Y, OBS_Z, OBS_R)
        self.robot_interface.publish_object_xyz("obstacle", x, y, z, r, 'sphere')

    def run_add_obstacle_change_goal_test(self, dyn_time):
        self.setup(self.start_state_dict['add_obstacle_change_goal'])
        rospy.loginfo("Running add_obstacle_change_goal test")
        rospy.loginfo("Sending initial joint goal")
        (initial_goal, final_goal) = self.goal_state_dict['add_obstacle_change_goal']
        self.new_goal_pub.publish(Float64MultiArray(self.layout, initial_goal))
        rospy.sleep(dyn_time)
        rospy.loginfo("Adding obstacle and then sending new goal")
        (x, y, z, size) = (0.8, 0.0, 0.4, (0.5, 0.02, 0.4))
        self.robot_interface.publish_object_xyz("obstacle", x, y, z, size)
        self.new_goal_pub.publish(Float64MultiArray(self.layout, final_goal))


if __name__ == "__main__":
    rospy.init_node("test_interface")
    ti = TestInterface()
    rospy.spin()
    # test = rospy.get_param('/test_interface/test', 'change_goal')
    # rospy.logwarn(f"TestInterface running test: {test}")
    # ti.run_test(test)
