#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from demo_interface import DemoInterface

JOINT_STATE1 = [0.00, -0.785, 0.00, -2.356, 0.00, 1.571, 0.785]
JOINT_STATE2 = [0.00, 0.00, 0.00, -1.75, 0.00, 1.571, 0.785]
JOINT_STATE3 = [0.00, 0.785, 0.00, -1.00, 0.00, 1.571, 0.785]
JOINT_STATES = [JOINT_STATE1, JOINT_STATE2, JOINT_STATE3]

class CollisionTest:

    def __init__(self):
        self.init_subs_pubs()
        self.demo_interface = DemoInterface(node_initialized=True)

    def init_subs_pubs(self):
        self.imminent_collision_sub = rospy.Subscriber("/collision_imminent", Bool, self.imminent_collision_cb)
        self.current_path_pub = rospy.Publisher("/current_path", JointTrajectory, queue_size=1)
        self.reached_state_pub = rospy.Publisher("/reached_state", JointTrajectoryPoint, queue_size=1)

    def imminent_collision_cb(self, bool_msg):
        rospy.loginfo(f"Collision immiment cb: {bool_msg.data}")

    def publish_sample_path(self):
        self.joint_trajectory = JointTrajectory()
        for joint_state in JOINT_STATES:
            joint_state_msg = JointTrajectoryPoint()
            for i in range(len(joint_state)):
                joint_state_msg.positions.append(joint_state[i])
            self.joint_trajectory.points.append(joint_state_msg)
        self.current_path_pub.publish(self.joint_trajectory)

    def publish_test_obstacle(self):
        self.demo_interface.publish_object_manual('obstacle', 0.5, 0.0, 0.6, 0.05, type='sphere')

    def remove_test_obstacle(self):
        self.demo_interface.remove_object('obstacle')

    def publish_reached_state(self):
        reached_state_msg = self.joint_trajectory.points[1]
        self.reached_state_pub.publish(reached_state_msg)


if __name__ == "__main__":
    rospy.init_node("collision_test_node")
    ct = CollisionTest()
    # sleep to allow pubs and subs time to connect
    rospy.sleep(1.0)
    input("press enter to publish sample path\n")
    ct.publish_sample_path()
    input("press enter to pubish test obstacle\n")
    ct.publish_test_obstacle()
    input("press enter to remove test obstacle\n")
    ct.remove_test_obstacle()
    input("press enter to publish reached next state\n")
    ct.publish_reached_state()

