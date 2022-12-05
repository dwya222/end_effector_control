#!/usr/bin/env python3

import copy

import rospy
import actionlib
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

START_JOINT_POSITIONS = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
# GOAL_JOINT_POSITIONS = [-0.5578778636322044, 0.908623569993187, -0.38844591131009487,
#                         -1.2094721531006272, 0.3444179383032919, 2.0541426810783356,
#                         -0.18421686175609792]
GOAL_JOINT_POSITIONS = [0.9022525451956217, -1.0005812062660042, -1.7602947518592436,
                        -2.7099525294963933, -0.1420045228964755, 3.493668294307763,
                        -0.3472854722693375]

if __name__ == "__main__":
    rospy.init_node("example_trajectory_client")
    trajectory_client = actionlib.SimpleActionClient('/execute_trajectory',
                                                     ExecuteTrajectoryAction)
    trajectory_client.wait_for_server()
    trajectory_msg1 = ExecuteTrajectoryGoal()

    trajectory_msg1.trajectory.joint_trajectory.joint_names = (
            ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
             'panda_joint6', 'panda_joint7'])
    trajectory_msg2 = copy.deepcopy(trajectory_msg1)

    point1 = JointTrajectoryPoint()
    point2 = JointTrajectoryPoint()

    for pos in START_JOINT_POSITIONS:
        point1.positions.append(pos)
        point1.velocities.append(0)

    for pos in GOAL_JOINT_POSITIONS:
        point2.positions.append(pos)
        point2.velocities.append(0)

    point21 = copy.deepcopy(point2)
    point12 = copy.deepcopy(point1)

    point21.time_from_start = rospy.Duration(2)
    point12.time_from_start = rospy.Duration(2)

    trajectory_msg1.trajectory.joint_trajectory.points.extend([point1, point21])
    trajectory_msg2.trajectory.joint_trajectory.points.extend([point2, point12])

    count = 0
    while not rospy.is_shutdown():
        input("Press enter to send next joint trajectory msg")
        if count % 2 == 0:
            trajectory_client.send_goal(trajectory_msg1)
        else:
            trajectory_client.send_goal(trajectory_msg2)
        count += 1


