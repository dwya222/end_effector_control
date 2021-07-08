#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from scipy.spatial.transform import Rotation as R
from display_point_publisher import point_pub


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
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

  def go_to_joint_state(self):
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    rospy.loginfo(joint_goal)
    [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    joint_goal[0] = 0
    joint_goal[1] = -0.785
    joint_goal[2] = 0
    joint_goal[3] = -2.356
    joint_goal[4] = 0
    joint_goal[5] = 1.571
    joint_goal[6] = 0.785
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self, rpy):
    move_group = self.move_group
    print(move_group.get_current_pose())
    print(move_group.get_current_rpy())
    pose_goal = geometry_msgs.msg.Pose()
    current_pose = move_group.get_current_pose()
    rpy_rot = R.from_euler('xyz', rpy, degrees=False)
    quat = rpy_rot.as_quat()
    print(quat)
    pose_goal.position.x = current_pose.pose.position.x
    pose_goal.position.y = current_pose.pose.position.y
    pose_goal.position.z = current_pose.pose.position.z
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    print(move_group.get_current_rpy())
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_rpy_goal(self, rpy):
    move_group = self.move_group
    print(move_group.get_current_pose())
    # start_rpy = [3.1415926535848926, -4.896669808048392e-12, -0.785000000006922]
    move_group.set_rpy_target(rpy)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    print("printing current pose")
    print(self.move_group.get_current_pose().pose)
    print("printing current rpy")
    rpy_new = self.move_group.get_current_rpy()
    for num,val in enumerate(rpy_new):
        rpy_new[num] = math.degrees(val)
    print(rpy_new)

  def follow_point(self, point):
      move_group = self.move_group
      pose_goal = geometry_msgs.msg.Pose()
      # Using horizontal ee orientation
      if point[2] >= 0.4 and point[2] <= 0.8:
          rpy = [-95, -45, -85]
          pose_goal.position.x = point[0] - .1
          pose_goal.position.y = point[1]
          pose_goal.position.z = point[2]
      if point[2] < 0.4:
          rpy = [145, 30, 125]
          pose_goal.position.x = point[0] - .1
          pose_goal.position.y = point[1]
          pose_goal.position.z = point[2] + .1
      if point[2] > 0.8:
          rpy = [-50, -40, -115]
          pose_goal.position.x = point[0] - .1
          pose_goal.position.y = point[1]
          pose_goal.position.z = point[2] - .05
      rpy_rot = R.from_euler('xyz', rpy, degrees=True)
      quat = rpy_rot.as_quat()
      pose_goal.orientation.x = quat[0]
      pose_goal.orientation.y = quat[1]
      pose_goal.orientation.z = quat[2]
      pose_goal.orientation.w = quat[3]
      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()
      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 0.01)

  def disp_point(self, point):
      point_publisher = point_pub()
      point_publisher.display_point(point)


def main():
  mover = MoveGroupPythonIntefaceTutorial()
  while True:
      x = input("Enter x position of point: ")
      if x=="start":
          mover.go_to_joint_state()
      else:
          y = input("Enter y position of point: ")
          z = input("Enter z position of point: ")
          point = [x,y,z]
          mover.disp_point(point)
          mover.follow_point(point)

if __name__ == '__main__':
  main()
