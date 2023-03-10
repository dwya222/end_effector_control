#!/usr/bin/env python3

import os
import pandas as pd

import rospy
from rospy.exceptions import ROSException
import rospkg
import actionlib
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, String, Bool, Int64
from trajectory_msgs.msg import JointTrajectory

from demo_interface import DemoInterface
from robo_demo_msgs.msg import JointTrajectoryPointStamped
import utils


# Database setup
rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
PLANNING_DATA_PATH = os.path.join(EE_CONTROL_PATH, 'data', 'planning')
DATA_PATH = os.path.join(PLANNING_DATA_PATH, 'maintenance_data.csv')
COLUMNS = ["Root Rewiring", "Cost", "Iterations"]
# Joint state setup
START_STATE = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
INITIAL_TARGET = [0.9022525451956217, -1.0005812062660042, -1.7602947518592436, -2.7099525294963933,
                 -0.1420045228964755, 3.493668294307763, -0.3472854722693375]
FINAL_TARGET = [1.1901980109241104, 0.9615559746057705, -0.5881359185350531, -1.2015471132200233,
               0.5281574185640393, 2.0160775068768824, 1.3658315499054479]


class ChangeGoalTestInterface():

    def __init__(self):
        self.init_db()
        self.init_setup()
        self.init_comms()
        rospy.sleep(0.2)

    def init_db(self):
        self.db_path = DATA_PATH
        self.db_columns = COLUMNS
        if os.path.exists(self.db_path):
            self.db = pd.read_csv(self.db_path)
            rospy.loginfo(f"Initialized saved db: \n{self.db}")
        else:
            self.db = pd.DataFrame(columns=self.db_columns)
            rospy.loginfo(f"Initialized new common db: \n{self.db}")

    def init_setup(self):
        self.robot_interface = DemoInterface(node_initialized=True)
        self.start_target = START_STATE
        self.initial_target = INITIAL_TARGET
        self.final_target = FINAL_TARGET
        self.layout = MultiArrayLayout()
        config_str = "/move_group/planner_configs/RTRRTstarkConfigDefault/"
        self.priming_secs = rospy.get_param(config_str + 'prime_tree_secs', 0.0)
        self.rewiring_enabled = rospy.get_param(config_str + 'enable_root_rewiring', 1)
        self.current_path = []
        self.waiting_for_first_solution = False
        self.waiting_for_first_solution_iter = False
        rospy.loginfo(f"priming secs: {self.priming_secs}, root rewiring: {self.rewiring_enabled}")

    def init_comms(self):
        self.executing_to_state_sub = rospy.Subscriber("/executing_to_state",
                                                       JointTrajectoryPointStamped,
                                                       self.executing_to_state_cb, queue_size=1)
        self.current_path_sub = rospy.Subscriber("/current_path", JointTrajectory,
                                                 self.current_path_cb, queue_size=1)
        self.solution_iter_sub = rospy.Subscriber("/solution_iterations", Int64,
                                                  self.solution_iter_cb, queue_size=1)
        self.new_goal_pub = rospy.Publisher("/new_planner_goal", Float64MultiArray, queue_size=1)
        self.initializing_test_pub = rospy.Publisher("/initializing_test", Bool, queue_size=1,
                                                     latch=True)
        self.test_service = rospy.Service("change_goal_test", Trigger, self.run_test)

    def executing_to_state_cb(self, state_msg):
        self.current_path.append(state_msg.trajectory_point.positions)

    def current_path_cb(self, path_msg):
        if self.waiting_for_first_solution:
            if len(path_msg.points) > 0 and utils.almost_equal(path_msg.points[-1].positions,
                    self.final_target):
                rospy.loginfo("Path satisfies goal")
                self.solve_time = (rospy.Time.now() - self.start_solve_time).to_sec()
                rospy.logwarn(f"took {self.solve_time} secs to find first solution path")
                self.waiting_for_first_solution = False
            else:
                rospy.logwarn("Path does not satisfy goal (expect this log once)")
                self.waiting_for_first_solution = True

    def solution_iter_cb(self, iter_msg):
        if self.waiting_for_first_solution_iter:
            self.first_solution_iter = iter_msg.data
            rospy.logwarn(f"took {self.first_solution_iter} iters to find first solution path")
            self.waiting_for_first_solution_iter = False

    def setup_test(self, start):
        self.initializing_test_pub.publish(Bool(True))
        rospy.loginfo(f"Setting to start: {start}")
        rospy.sleep(1.0)
        current_state = self.robot_interface.move_group.get_current_joint_values()
        if utils.almost_equal(current_state, start):
            rospy.loginfo("Already at start, still publishing and waiting 5 sec")
            self.new_goal_pub.publish(Float64MultiArray(self.layout, start))
            rospy.sleep(5.0 + self.priming_secs)
        else:
            rospy.loginfo("Publishing start as goal and waiting for goal_achieved")
            self.new_goal_pub.publish(Float64MultiArray(self.layout, start))
            try:
                goal_achieved_msg = rospy.wait_for_message("/goal_achieved", Bool, timeout=20.0)
            except ROSException as e:
                rospy.logerr(f"Exception recieved for timeout: {e}")
                self.new_goal_pub.publish(Float64MultiArray(self.layout, start))
                goal_achieved_msg = rospy.wait_for_message("/goal_achieved", Bool, timeout=20.0)
            rospy.loginfo(f"Start achieved: {goal_achieved_msg.data}")
        self.initializing_test_pub.publish(Bool(False))

    def run_test(self, trigger_request, retry_count=0):
        rospy.loginfo(f"Attempting to run change goal test, retry: {retry_count}")
        try:
            rospy.sleep(0.5)
            self.setup_test(self.start_target)
            rospy.loginfo("Test successfully setup, sleeping 1 second and then running test")
            rospy.sleep(1.0)
            self.current_path = []
            self.run_change_goal_test()
        except Exception as e:
            rospy.logerr(f"Encountered exception: {e}")
            return TriggerResponse(success=False)
        rospy.loginfo("Waiting for goal achieved")
        try:
            goal_achieved = rospy.wait_for_message("/goal_achieved", Bool, timeout=15.0)
            rospy.loginfo(f"Successfully ran change goal test")
            self.store_data(True)
            return TriggerResponse(success=True)
        except ROSException as e:
            if rospy.is_shutdown():
                raise Exception
            if retry_count < 2:
                retry_count += 1
                rospy.logerr(f"Timed out waiting for change goal test to complete, retrying")
                return self.run_test(TriggerRequest(), retry_count=retry_count)
            if self.name == "MRRTstar":
                self.store_data(False)
            return TriggerResponse(success=False, message="Encountered exception waiting for goal "
                                                          f"achieved: {e}")

    def run_change_goal_test(self):
        rospy.loginfo("Starting primed PRTRRTstar planning")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.initial_target))
        self.start_solve_time = rospy.Time.now() + rospy.Duration(self.priming_secs)
        rospy.sleep(self.priming_secs + 2.25)
        rospy.loginfo("Sending final joint goal")
        self.new_goal_pub.publish(Float64MultiArray(self.layout, self.final_target))
        self.waiting_for_first_solution = True
        self.waiting_for_first_solution_iter = True

    def store_data(self, goal_achieved):
        cost = utils.path_cost(self.current_path)
        db_entry = pd.DataFrame([[self.rewiring_enabled, cost, self.first_solution_iter]],
                                columns=self.db_columns)
        rospy.loginfo(f"Saving path info to...{self.db_path}")
        rospy.loginfo(f"db_entry:\n{db_entry}")
        rospy.loginfo("-----------------------------------------")
        self.db = pd.concat([self.db, db_entry])
        self.db.to_csv(self.db_path, index=False)

if __name__ == "__main__":
    rospy.init_node("change_goal_test_interface")
    ti = ChangeGoalTestInterface()
    rospy.spin()
