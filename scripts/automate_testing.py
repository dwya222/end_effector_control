#!/usr/bin/env python3

import os
import sys

import roslaunch
import rosnode
import rospkg
import rospy

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
EE_CONTROL_LAUNCH_PATH = os.path.join(EE_CONTROL_PATH, 'launch')
TEST_NODE = "/test_interface_node"
DEFAULT_TEST = "add_obstacle_rrt_only"
if len(sys.argv) > 1:
    TEST_TYPE = sys.argv[1]
else:
    rospy.logwarn(f"No test type specified, defaulting to {DEFAULT_TEST}")
    TEST_TYPE = DEFAULT_TEST

if __name__ == "__main__":
    rospy.init_node("automate_test_interface")
    rospy.sleep(2)
    launch_path = os.path.join(EE_CONTROL_LAUNCH_PATH, 'run_cost_test.launch')
    cli_args = [launch_path, f'test:={TEST_TYPE}']
    launch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]

    iterations = int(input("How many iterations would you like to run?\n"))
    rospy.loginfo(f"Running test for '{iterations}' iterations")
    rospy.sleep(2)
    for i in range(iterations):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

        launch.start()
        rospy.sleep(2)

        try:
            rosnode.rosnode_ping(TEST_NODE)
        except Exception as e:
            rospy.loginfo(f"Caught exception: {e}")
        rospy.sleep(0.5)
        rospy.loginfo("Stopping launch file from test automater")
        launch.shutdown()
