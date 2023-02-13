#!/usr/bin/env python3

import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robo_demo_msgs.msg import JointTrajectoryPointStamped, JointTrajectoryPointClearStamped
from demo_interface import DemoInterface

JOINT_STATE1 = [0.00, -0.785, 0.00, -2.356, 0.00, 1.571, 0.785]
# JOINT_STATE2 = [0.00, 0.00, 0.00, -1.75, 0.00, 1.571, 0.785]
JOINT_STATE2 = [0.40920392655323845, -0.45299422225870184, -1.3031637768785265,
                -2.1619692963673867, -0.2533941856117664, 3.0280971371667045, -0.3360878201743867]
# JOINT_STATE3 = [0.00, 0.785, 0.00, -1.00, 0.00, 1.571, 0.785]
JOINT_STATE3 = [0.9022255518660315, -1.0005112966461382, -1.760256825506428, -2.709892552480913,
                -0.14203649926382245, 3.493591515981207, -0.34730963534631754]
JOINT_STATES = [JOINT_STATE1, JOINT_STATE2, JOINT_STATE3]

class CollisionTest:

    def __init__(self):
        self.init_subs_pubs()
        self.demo_interface = DemoInterface(node_initialized=True)

    def init_subs_pubs(self):
        self.edge_clear_sub = rospy.Subscriber("/edge_clear", JointTrajectoryPointClearStamped,
                                               self.edge_clear_cb)
        self.current_path_pub = rospy.Publisher("/current_path", JointTrajectory, queue_size=1)
        self.executing_to_state_pub = rospy.Publisher("/executing_to_state",
                                                      JointTrajectoryPointStamped, queue_size=1)

    def edge_clear_cb(self, point_stamped_msg):
        rospy.loginfo(f"Collision immiment cb: {point_stamped_msg}")

    def publish_sample_path(self):
        self.joint_trajectory = JointTrajectory()
        for joint_state in JOINT_STATES:
            joint_state_msg = JointTrajectoryPoint()
            for i in range(len(joint_state)):
                joint_state_msg.positions.append(joint_state[i])
            self.joint_trajectory.points.append(joint_state_msg)
        self.current_path_pub.publish(self.joint_trajectory)

    def publish_test_obstacle(self):
        # self.demo_interface.publish_object_xyz('obstacle', 0.5, 0.0, 0.6, 0.05, primitive='sphere')
        self.demo_interface.publish_object_xyz('obstacle', 0.4, -0.3, 0.4, 0.05, primitive='sphere')

    def remove_test_obstacle(self):
        self.demo_interface.remove_object('obstacle')

    def publish_executing_to_state(self):
        executing_to_state_msg = JointTrajectoryPointStamped()
        executing_to_state_msg.trajectory_point = self.joint_trajectory.points[1]
        self.executing_to_state_pub.publish(executing_to_state_msg)


if __name__ == "__main__":
    rospy.init_node("collision_test_node")
    ct = CollisionTest()
    # sleep to allow pubs and subs time to connect
    rospy.sleep(1.0)
    input("press enter to publish sample path\n")
    ct.publish_sample_path()
    input("press enter to pubish test obstacle\n")
    ct.publish_test_obstacle()
    input("press enter to publish executing to state\n")
    ct.publish_executing_to_state()
    input("press enter to remove test obstacle\n")
    ct.remove_test_obstacle()
