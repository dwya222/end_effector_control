#!/usr/bin/env python3

import sys
import yaml

import rospy

from demo_interface import DemoInterface
from sensor_msgs.msg import JointState

def parse_joint_goal(filename):
    with open(filename, "r") as f:
        return yaml.safe_load(f)

if __name__ == "__main__":
    rospy.init_node("joint_state_publisher")
    joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    joint_goal = JointState()
    joint_goal.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                       'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1',
                       'panda_finger_joint2']
    if len(sys.argv) > 1:
        joint_goal.position = parse_joint_goal(sys.argv[1])
        joint_goal.position.extend([0.035, 0.035])
    else:
        for i in range(7):
            joint_goal.append(float(input(f"Input joint goal for joint {i}: ")))
        joint_goal.position.extend([0.035, 0.035])
    print(f"Joint goal: {joint_goal}")
    # sleep so subscribers have time to connect to publisher
    rospy.sleep(0.5)
    joint_state_pub.publish(joint_goal)
