#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from scipy.spatial.transform import Rotation as R


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

def euler(pose):
    new_rot = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    output = np.array(new_rot.as_euler('xyz', degrees=True))
    return output

def main():
  mover = MoveGroupPythonIntefaceTutorial()
  np.set_printoptions(suppress=True)
  while True:
       raw_input("Type enter to see next pose \n")
       current_pose = mover.move_group.get_current_pose().pose
       current_joint_values = mover.move_group.get_current_joint_values()
       print("joint state info:")
       print(current_joint_values)
       print("pose info:")
       print(current_pose)
       output = euler(current_pose)
       print("Euler [x, y, z]:")
       print(output)


if __name__ == '__main__':
  main()
