#!/usr/bin/env python3

import rospy
import actionlib

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


PANDA_JOINTS = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5",
                "panda_joint6", "panda_joint7"]
START_POSITIONS = [0.00, -0.785, 0.00, -2.356, 0.00, 1.571, 0.785]
POSITION1 = [0.9022525451956217, -1.0005812062660042, -1.7602947518592436, -2.7099525294963933,
              -0.1420045228964755, 3.493668294307763, -0.3472854722693375]
POSITOIN2 = [0.893870439461073, -1.607361293022906, -1.6443782870864856, -1.9440227758046293,
              -0.9470311283784122, 2.69534351952731, 0.0315925159228614]
ZEROS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POSITIONS = [START_POSITIONS, POSITION1, POSITOIN2]
CMD_TOPIC = "/position_joint_trajectory_controller/command"
CONTROLLER_TOPIC = "/position_joint_trajectory_controller/follow_joint_trajectory"
CLIENT_MODE = False

def create_goal(positions):
    msg = FollowJointTrajectoryGoal()
    # Stamp of 0 means begin immediately
    msg.trajectory.header.stamp = rospy.Time(0)
    msg.trajectory.joint_names = PANDA_JOINTS
    point_msg = JointTrajectoryPoint()
    point_msg.time_from_start = rospy.Duration(5.0)
    point_msg.positions = positions
    point_msg.velocities = ZEROS
    msg.trajectory.points.append(point_msg)
    return msg

def create_msg(positions):
    msg = JointTrajectory()
    # Stamp of 0 means begin immediately
    msg.header.stamp = rospy.Time(0)
    msg.joint_names = PANDA_JOINTS
    point_msg = JointTrajectoryPoint()
    point_msg.time_from_start = rospy.Duration(5.0)
    point_msg.positions = positions
    point_msg.velocities = ZEROS
    msg.points.append(point_msg)
    return msg

if __name__ == "__main__":
    rospy.init_node('trajectory_command_node')
    goals = []
    msgs = []
    for position in POSITIONS:
        goals.append(create_goal(position))
        msgs.append(create_msg(position))
    i = 0
    if CLIENT_MODE:
        trajectory_client = actionlib.SimpleActionClient(CONTROLLER_TOPIC,
                                                         FollowJointTrajectoryAction)
        while not trajectory_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo(f"Waiting for the {CONTROLLER_TOPIC} action server")
        while not rospy.is_shutdown():
            input("Press Enter to send next trajectory point")
            trajectory_client.send_goal(goals[i % 3])
            i += 1
    else:
        cmd_pub = rospy.Publisher(CMD_TOPIC, JointTrajectory, queue_size=1)
        while not rospy.is_shutdown():
            input("Press Enter to send next trajectory point")
            cmd_pub.publish(msgs[i % 3])
            i += 1
