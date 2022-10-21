#!/usr/bin/env python3

import rospy
from demo_interface import DemoInterface
from geometry_msgs.msg import Point

DEBUG = False

def main():
  mover = DemoInterface()
  point_msg = Point()
  start = "start"
  n = 'n'
  y = 'y'
  while not rospy.is_shutdown():
      if DEBUG:
          point_msg.x = input("Enter x position of point (or start): ")
          if point_msg.x=="start":
              mover.go_to_start()
          else:
              point_msg.y = input("Enter y position of point: ")
              point_msg.z = input("Enter z position of point: ")
              mover.planning_test(point_msg)
      else:
          input("Press enter to plan to default point (0.4, -0.4, 0.6)")
          point_msg.x = 0.5
          point_msg.y = -0.4
          point_msg.z = 0.6
          mover.planning_test(point_msg)
          break


if __name__ == '__main__':
  main()
