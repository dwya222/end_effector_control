#!/usr/bin/env python3

import sys

import rospy
from std_msgs.msg import Float64MultiArray

from demo_interface import DemoInterface
from RRT_dynamic_interface import RRTPlannerControlInterface

# Create default joint goals here
# DEFAULT_INITIAL_GOAL = [...]
# DEFAULT_FINAL_GOAL = [...]
# required base argments for a rosrun: rosrun [pkg_name] [node_executable]
BASE_AMOUNT_ROSRUN_ARGS = 3

def parse_joint_goal(filename):
    with open(filename, "r") as f:
        return yaml.safe_load(f)

if __name__ == "__main__":
    rospy.init_node("change_goal_node")
    joint_goals = []
    # TODO: may want to change 
    # if this node is run with rosrun
    if sys.argv[0] == 'rosrun':
        # Need at least 2 joint goals to run test (initial and changed)
        if len(sys.argv) >= (BASE_AMOUNT_ROSRUN_ARGS + 2):
            for file_num in range(len(sys.argv) - BASE_AMOUNT_ROSRUN_ARGS):
                joint_goals.append(parse_joint_goal(sys.argv[3 + file_num]))
        else:
            rospy.loginfo("Not enough joint_goal files provided (need at least 2), using defaults")
            joint_goals.extend([DEFAULT_INITIAL_GOAL, DEFAULT_FINAL_GOAL])
    # otherwise assuming it is run from a launch file
    else:
        joint_goals.append(rospy.get_param("/change_goal_node/initial_joint_goal",
                                           DEFAULT_INITIAL_GOAL))
        joint_goals.append(rospy.get_param("/change_goal_node/final_joint_goal",
                                           DEFAULT_FINAL_GOAL))
        rospy.logerr("No joint goal provided, exiting")
        raise BaseException()
    print(f"Joint goals:\n{joint_goals}")

    new_goal_pub = rospy.Publisher("new_planner_goal", Float64MultiArray, 10)

    joint_goal_msgs = []
    array_msg = Float64MultiArray()
    for joint_goal in joint_goal_msgs:
        array_msg.data = joint_goal
        joint_goal_msgs.append(array_msg)

    pub_count = 0
    while not rospy.is_shutdown():


  int pub_count = 0;
  while (ros::ok())
  {
    ROS_INFO("Press Enter to publish another message");
    std::cin.get();
    if (pub_count%2 == 0)
    {
      new_goal_pub.publish(new_goal_msg);
      ROS_INFO("Published msg1");
    }
    else
    {
      new_goal_pub.publish(new_goal_msg2);
      ROS_INFO("Published msg2");
    }
    pub_count++;
  }
}

