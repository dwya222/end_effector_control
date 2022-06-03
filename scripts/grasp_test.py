#!/usr/bin/env python3

from demo_interface import DemoInterface
from geometry_msgs.msg import Point
import rospy
import rospy
import actionlib
from franka_gripper.msg import MoveGoal, MoveAction

d = DemoInterface()
d.go_to_start()
client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
client.wait_for_server()
close_goal = MoveGoal(width = 0.045, speed = 0.08)
open_goal = MoveGoal(width = 0.08, speed = 0.08)
client.send_goal(open_goal)
client.wait_for_result(rospy.Duration.from_sec(5.0))
point = Point()
point.x = 0.476445506302
point.y = 0.290862530436
point.z = 0.349948678975
d.follow_point(point, grasp=True)
rospy.sleep(1)
point.x = 0.576445506302
point.y = 0.290862530436
point.z = 0.349948678975
d.follow_point(point, grasp=True)
rospy.sleep(1)
client.send_goal(close_goal)
client.wait_for_result(rospy.Duration.from_sec(5.0))
rospy.sleep(1)
point.x = 0.576445506302
point.y = 0.290862530436
point.z = 0.649948678975
d.follow_point(point, grasp=True)
rospy.sleep(1)
point.x = 0.576445506302
point.y = 0.290862530436
point.z = 0.349948678975
d.follow_point(point, grasp=True)
client.send_goal(open_goal)
client.wait_for_result(rospy.Duration.from_sec(5.0))
point.x = 0.576445506302
point.y = 0.290862530436
point.z = 0.649948678975
d.follow_point(point, grasp=True)
d.go_to_start()
