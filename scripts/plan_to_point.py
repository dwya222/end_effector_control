#!/usr/bin/env python3

import rospy
from demo_interface import DemoInterface
from geometry_msgs.msg import Point

DEBUG = True

def main():
  mover = DemoInterface()
  point_msg = Point()
  start = "start"
  n = 'n'
  y = 'y'
  while not rospy.is_shutdown():
      if DEBUG:
          init = input("Enter x position of point (or start): ")
          if init=="start":
              mover.go_to_start()
          else:
              point_msg.x = float(init)
              point_msg.y = float(input("Enter y position of point: "))
              point_msg.z = float(input("Enter z position of point: "))
              mover.planning_test(point_msg)
      else:
          input("Press enter to plan to default point (0.5, -0.4, 0.6)")
          point_msg.x = 0.5
          point_msg.y = -0.4
          point_msg.z = 0.6
          mover.planning_test(point_msg)
          break


if __name__ == '__main__':
  main()
