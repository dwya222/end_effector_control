#!/usr/bin/env python3

import json
import os
import numpy as np
import pandas as pd
import sys
import threading as th
from abc import abstractmethod

import rospy
from rospy.exceptions import ROSException
import actionlib
import rospkg
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, String, Bool
from trajectory_msgs.msg import JointTrajectory

from demo_interface import DemoInterface
from robo_demo_msgs.srv import RunPlanningTest, RunPlanningTestResponse
from robo_demo_msgs.msg import JointTrajectoryPointStamped
import utils

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
PRTRRT_DATA_PATH = os.path.join(PLANNING_DATA_PATH, 'PRTRRT_data.csv')
PRTRRT_JSON_PATH = os.path.join(PLANNING_DATA_PATH, 'PRTRRT_data.json')
PRTRRT_COLUMNS = ["Scenario", "Change State", "Cost After Change", "Path"]
MRRT_DATA_PATH = os.path.join(PLANNING_DATA_PATH, 'MRRT_data.csv')
MRRT_COLUMNS = ["Scenario", "Cost After Change", "Path"]
COMMON_DATA_PATH = os.path.join(PLANNING_DATA_PATH, 'planning_data.csv')
COMMON_COLUMNS = ["Scenario", "PRTRRT Cost", "MRRT Cost", "PRTRRT Time", "MRRT Time",
                  "PRTRRT Retry", "MRRT Retry"]
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
        self.init_common_db()
        self.init_db()
        self.init_setup()
        self.init_comms()
        rospy.sleep(0.2)

    def init_common_db(self):
        self.common_db_path = COMMON_DATA_PATH
        self.common_db_columns = COMMON_COLUMNS
        if os.path.exists(self.common_db_path):
            self.common_db = pd.read_csv(self.common_db_path)
            rospy.loginfo(f"Initialized saved common db: \n{self.common_db}")
        else:
            self.common_db = pd.DataFrame(columns=self.common_db_columns)
            rospy.loginfo(f"Initialized new common db: \n{self.common_db}")

    @abstractmethod
    def init_db(self):
        pass

    def init_setup(self):
        self.robot_interface = DemoInterface(node_initialized=True)
        self.start_state_dict = {'change_goal': START_STATE1,
                                 'add_obstacle': START_STATE2,
                                 'add_obstacle_change_goal': START_STATE1}
        self.goal_state_dict = {'change_goal': (INITIAL_GOAL1, FINAL_GOAL1),
                                'add_obstacle': (INITIAL_GOAL2, None),
                                'add_obstacle_change_goal': (INITIAL_GOAL3, FINAL_GOAL1)}
        self.dyn_time_dict = {'change_goal': 1.75,
                              'add_obstacle': 0.75,
                              'add_obstacle_change_goal': 2.0}
        self.test_dict = {'change_goal': self.run_change_goal_test,
                          'add_obstacle': self.run_add_obstacle_test,
                          'add_obstacle_change_goal': self.run_add_obstacle_change_goal_test}
        self.layout = MultiArrayLayout()
        self.priming_secs = rospy.get_param('/move_group/planner_configs/RTRRTstarkConfigDefault/'
                'prime_tree_secs', 0.0)
        self.current_path = []
        self.waiting_for_first_solution = False
        rospy.loginfo(f"priming secs: {self.priming_secs}")

    def init_comms(self):
        self.executing_to_state_sub = rospy.Subscriber("/executing_to_state",
                                                       JointTrajectoryPointStamped,
                                                       self.executing_to_state_cb, queue_size=1)
        self.current_path_sub = rospy.Subscriber("/current_path", JointTrajectory,
                                                 self.current_path_cb, queue_size=1)
        self.new_goal_pub = rospy.Publisher("/new_planner_goal", Float64MultiArray, queue_size=1)
        self.initializing_test_pub = rospy.Publisher("/initializing_test", Bool, queue_size=1,
                                                     latch=True)

    def executing_to_state_cb(self, state_msg):
        self.current_path.append(state_msg.trajectory_point.positions)

    def current_path_cb(self, path_msg):
        if self.waiting_for_first_solution:
            self.solve_time = (rospy.Time.now() - self.start_solve_time).to_sec()
            rospy.logwarn(f"took {self.solve_time} to find first solution path")
            if utils.almost_equal(path_msg.points[-1].positions, self.final_goal):
                rospy.loginfo("Path satisfies goal")
            else:
                rospy.logerr("Path does not satisfy goal")
            self.waiting_for_first_solution = False

    def setup_test(self, start):
        self.initializing_test_pub.publish(Bool(True))
        self.robot_interface.remove_object("obstacle")
        # self.object_names = self.robot_interface.scene.get_known_object_names()
        # for object_name in self.object_names:
        #     self.robot_interface.remove_object(object_name)
        rospy.loginfo(f"Setting to start: {start}")
        current_state = self.robot_interface.move_group.get_current_joint_values()
        if utils.almost_equal(current_state, start):
            rospy.loginfo("Already at start, still publishing and waiting 5 sec")
            self.new_goal_pub.publish(Float64MultiArray(self.layout, start))
            rospy.sleep(5.0 + self.priming_secs)
        else:
            rospy.loginfo("Publishing start as goal and waiting for goal_achieved")
            self.new_goal_pub.publish(Float64MultiArray(self.layout, start))
            goal_achieved_msg = rospy.wait_for_message("/goal_achieved", Bool)
            rospy.loginfo(f"Start achieved: {goal_achieved_msg.data}")
        self.initializing_test_pub.publish(Bool(False))

    def run_test(self, scenario, start_state, retry_count=0):
        self.retry_count = retry_count
        rospy.loginfo(f"Attempting to run '{scenario}' test, retry: {retry_count}")
        try:
            rospy.sleep(0.5)
            self.setup_test(start_state)
            (self.initial_goal, self.final_goal) = self.goal_state_dict[scenario]
            self.current_path = []
            self.test_dict[scenario]()
        except Exception as e:
            rospy.logerr(f"Encountered exception: {e}")
            return RunPlanningTestResponse(success=False)
        rospy.loginfo("Waiting for goal achieved")
        try:
            goal_achieved = rospy.wait_for_message("/goal_achieved", Bool, timeout=30.0)
            rospy.loginfo(f"Successfully ran '{scenario}' test")
            self.store_data(scenario, True)
            return RunPlanningTestResponse(success=True)
        except ROSException as e:
            if rospy.is_shutdown():
                raise Exception
            if self.retry_count < 2:
                self.retry_count += 1
                rospy.logerr(f"Timed out waiting for '{scenario}' test to complete, retrying")
                return self.run_test(scenario, start_state, self.retry_count)
            if self.name == "MRRTstar":
                self.store_data(scenario, False)
            return RunPlanningTestResponse(success=False, message=f"Encountered exception waiting "
                                                                   "for goal achieved: {e}")

    def add_ball(self):
        rospy.loginfo("Adding ball obstacle")
        (x, y, z, r) = (OBS_X, OBS_Y, OBS_Z, OBS_R)
        self.robot_interface.publish_object_xyz("obstacle", x, y, z, r, 'sphere')

    def add_wall(self):
        rospy.loginfo("Adding wall obstacle")
        (x, y, z, size) = (0.8, 0.0, 0.4, (0.5, 0.02, 0.4))
        self.robot_interface.publish_object_xyz("obstacle", x, y, z, size)

    @abstractmethod
    def run_change_goal_test(self):
        pass

    @abstractmethod
    def run_add_obstacle_test(self):
        pass

    @abstractmethod
    def run_add_obstacle_change_goal_test(self):
        pass

    @abstractmethod
    def store_data(self, scenario, goal_achieved):
        pass

    @abstractmethod
    def store_common_data(self, scenario, cost_after_change):
        pass


class TestInterfacePRTRRTstar(TestInterface):

# Tests will be run via the test_interface_service.

    def __init__(self):
        super().__init__()
        self.name = "PRTRRTstar"
        self.init_json()
        rospy.loginfo("TestInterface initialized for PRTRRTstar process")

    def init_db(self):
        self.db_path = PRTRRT_DATA_PATH
        self.db_columns = PRTRRT_COLUMNS
        if os.path.exists(self.db_path):
            self.db = pd.read_csv(self.db_path)
            rospy.loginfo(f"Initialized saved PRTRRT db: \n{self.db}")
        else:
            self.db = pd.DataFrame(columns=self.db_columns)
            rospy.loginfo(f"Initialized new PRTRRT db: \n{self.db}")

    def init_comms(self):
        super().init_comms()
        self.test_service = rospy.Service("test_interface_service", RunPlanningTest,
                                          self.run_planning_test)

    def init_json(self):
        self.json_path = PRTRRT_JSON_PATH
        if os.path.exists(self.json_path):
            with open(self.json_path, 'r') as f:
                self.json_data = json.load(f)
            rospy.loginfo(f"Initialized saved PRTRRT json data: \n{self.json_data}")
        else:
            self.json_data = {'Scenarios': [], 'Change States': []}
            rospy.loginfo(f"Initialized new PRTRRT json data: \n{self.json_data}")

    def run_planning_test(self, test_msg):
        rospy.loginfo(f"Attempting to run '{test_msg.scenario}' test")
        scenario = test_msg.scenario
        start_state = self.start_state_dict[test_msg.scenario]
        return self.run_test(scenario, start_state)

    def run_change_goal_test(self):
        rospy.loginfo("Sending initial joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.initial_goal))
        rospy.sleep(self.dyn_time_dict['change_goal'] + self.priming_secs)
        self.change_idx = len(self.current_path) - 1
        rospy.loginfo("Sending final joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.final_goal))
        self.waiting_for_first_solution = True
        self.start_solve_time = rospy.Time.now()

    def run_add_obstacle_test(self):
        rospy.loginfo("Sending initial joint goal")
        self.final_goal = self.initial_goal
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.initial_goal))
        rospy.sleep(self.dyn_time_dict['add_obstacle'] + self.priming_secs)
        self.change_idx = len(self.current_path) - 1
        self.add_ball()
        self.waiting_for_first_solution = True
        self.start_solve_time = rospy.Time.now()

    def run_add_obstacle_change_goal_test(self):
        rospy.loginfo("Sending initial joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.initial_goal))
        rospy.sleep(self.dyn_time_dict['add_obstacle_change_goal'] + self.priming_secs)
        self.change_idx = len(self.current_path) - 1
        self.add_wall()
        rospy.loginfo("Sending new goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.final_goal))
        self.waiting_for_first_solution = True
        self.start_solve_time = rospy.Time.now()

    def store_data(self, scenario, goal_achieved):
        cost_after_change = utils.path_cost(self.current_path[self.change_idx:])
        db_entry = pd.DataFrame([[scenario, self.current_path[self.change_idx], cost_after_change,
                                  self.current_path]], columns=self.db_columns)
        rospy.loginfo(f"Saving path info to...{self.db_path}")
        rospy.loginfo(f"db_entry:\n{db_entry}")
        rospy.loginfo("-----------------------------------------")
        self.db = pd.concat([self.db, db_entry])
        self.db.to_csv(self.db_path, index=False)
        # store common data
        self.store_common_data(scenario, cost_after_change)
        # store json data
        self.json_data['Scenarios'].append(scenario)
        self.json_data['Change States'].append(self.current_path[self.change_idx])
        with open(self.json_path, 'w') as f:
            json.dump(self.json_data, f, indent=2)

    def store_common_data(self, scenario, cost_after_change):
        db_entry = pd.DataFrame([[scenario, cost_after_change, None, self.solve_time, None,
                                  self.retry_count, None]], columns=self.common_db_columns)
        rospy.loginfo(f"Saving common path info to...{self.common_db_path}")
        rospy.loginfo(f"db_entry:\n{db_entry}")
        rospy.loginfo("-----------------------------------------")
        self.common_db = pd.concat([self.common_db, db_entry])
        self.common_db.to_csv(self.common_db_path, index=False)



class TestInterfaceMRRTstar(TestInterface):

# Tests will be run according to what is stored by the PRTRRTstar runs. One test needs to be run as
# a comparison to each test stored by an PRTRRTstar run.

    def __init__(self):
        super().__init__()
        self.name = "MRRTstar"
        self.load_prtrrt_json()
        rospy.loginfo("TestInterface initialized for MRRTstar process")

    def init_db(self):
        self.db_path = MRRT_DATA_PATH
        self.db_columns = MRRT_COLUMNS
        if os.path.exists(self.db_path):
            self.db = pd.read_csv(self.db_path)
            rospy.loginfo(f"Initialized saved MRRT db: \n{self.db}")
        else:
            self.db = pd.DataFrame(columns=self.db_columns)
            rospy.loginfo(f"Initialized new MRRT db: \n{self.db}")

    def load_prtrrt_json(self):
        try:
            with open(PRTRRT_JSON_PATH, 'r') as f:
                self.prtrrt_data = json.load(f)
        except:
            rospy.logerr("No PRTRRT data stored. Run PRTRRT tests first.")
        rospy.loginfo(f"Running tests: {self.prtrrt_data}")
        self.common_db_row = 0
        total_tests = len(self.prtrrt_data['Scenarios'])
        for scenario, state in zip(self.prtrrt_data['Scenarios'],
                                   self.prtrrt_data['Change States']):
            if rospy.is_shutdown():
                break
            # rospy.logwarn(f"entry: \n{self.common_db.at[self.common_db_row, 'MRRT Cost']}\n, entry"
            #               f"type: \n{type(self.common_db.at[self.common_db_row, 'MRRT Cost'])}")
            if not np.isnan(self.common_db.at[self.common_db_row, 'MRRT Cost']):
                rospy.loginfo(f"Skipping: {self.common_db.at[self.common_db_row, 'MRRT Cost']}")
                self.common_db_row += 1
                continue
            rospy.logwarn(f"Running test {self.common_db_row+1} / {total_tests}")
            response = self.run_test(scenario, state)
            self.common_db_row += 1
            if response.success:
                rospy.loginfo(f"Test succeeded for scenario: {scenario}, state: {state}")
            else:
                rospy.logerr(f"Test failed for scenario: {scenario}, state: {state}")

    def run_change_goal_test(self):
        rospy.loginfo("Sending joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.final_goal))
        self.waiting_for_first_solution = True
        self.start_solve_time = rospy.Time.now()

    def run_add_obstacle_test(self):
        self.add_ball()
        rospy.loginfo("Sending joint goal")
        self.final_goal = self.initial_goal
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.initial_goal))
        self.waiting_for_first_solution = True
        self.start_solve_time = rospy.Time.now()

    def run_add_obstacle_change_goal_test(self):
        self.add_wall()
        rospy.loginfo("Sending joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.final_goal))
        self.waiting_for_first_solution = True
        self.start_solve_time = rospy.Time.now()

    def store_data(self, scenario, goal_achieved):
        cost_after_change = utils.path_cost(self.current_path)
        db_entry = pd.DataFrame([[scenario, cost_after_change, self.current_path]],
                                columns=self.db_columns)
        rospy.loginfo(f"Saving path info to...{self.db_path}")
        rospy.loginfo(f"db_entry:\n{db_entry}")
        rospy.loginfo("-----------------------------------------")
        self.db = pd.concat([self.db, db_entry])
        self.db.to_csv(self.db_path, index=False)
        # store common data
        if goal_achieved:
            self.store_common_data(scenario, cost_after_change)
        else:
            self.store_common_data(scenario, 9999.99)

    def store_common_data(self, scenario, cost_after_change):
        rospy.loginfo(f"Saving common path info to...{self.common_db_path}")
        rospy.loginfo("-----------------------------------------")
        self.common_db.at[self.common_db_row, 'MRRT Cost'] = cost_after_change
        self.common_db.at[self.common_db_row, 'MRRT Time'] = self.solve_time
        self.common_db.at[self.common_db_row, 'MRRT Retry'] = self.retry_count
        self.common_db.to_csv(self.common_db_path, index=False)


if __name__ == "__main__":
    rospy.init_node("test_interface")
    try:
        planning_process = rospy.get_param('/planning_process')
    except KeyError as e:
        rospy.logerr(f"No 'planning_process param', make sure to initialize controller first:\n{e}")
    if planning_process == "M-RRTstar":
        ti = TestInterfaceMRRTstar()
    elif planning_process == "PRT-RRTstar":
        ti = TestInterfacePRTRRTstar()
    else:
        rospy.logerr(f"planning_process: '{planning_process}' does not match known processes")
    rospy.spin()
