#!/usr/bin/env python3

from demo_interface import DemoInterface
from geometry_msgs.msg import Point
import rospy
import rospy
import actionlib
from franka_gripper.msg import MoveGoal, MoveAction

d = DemoInterface()
client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
client.wait_for_server()
close_goal = MoveGoal(width = 0.052, speed = 0.08)
open_goal = MoveGoal(width = 0.08, speed = 0.08)
client.send_goal(open_goal)
client.wait_for_result(rospy.Duration.from_sec(5.0))
rospy.sleep(3.0)
# client.send_goal(close_goal)
client.wait_for_result(rospy.Duration.from_sec(5.0))
