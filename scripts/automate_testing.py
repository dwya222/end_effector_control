#!/usr/bin/env python3

import os

import roslaunch
import rosnode
import rospkg
import rospy

rospack = rospkg.RosPack()
EE_CONTROL_PATH = rospack.get_path('end_effector_control')
EE_CONTROL_LAUNCH_PATH = os.path.join(EE_CONTROL_PATH, 'launch')
TEST_NODE = "/test_interface_node"

if __name__ == "__main__":
    rospy.init_node("automate_test_interface")
    rospy.sleep(2)
    launch_path = os.path.join(EE_CONTROL_LAUNCH_PATH, 'run_cost_test.launch')
    cli_args = [launch_path, 'test:=add_obstacle_rrt_only']
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

        while rosnode.rosnode_ping(TEST_NODE, max_count=1):
            rospy.sleep(0.5)
        rospy.loginfo("rosnode died and caused no error")
        launch.shutdown()
