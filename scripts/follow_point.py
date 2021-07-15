#!/usr/bin/env python

from demo_interface import DemoInterface
from geometry_msgs.msg import Point

def main():
  mover = DemoInterface()
  point_msg = Point()
  start = "start"
  n = 'n'
  y = 'y'
  while True:
      point_msg.x = input("Enter x position of point (or start): ")
      if point_msg.x=="start":
          mover.go_to_start()
      else:
          point_msg.y = input("Enter y position of point: ")
          point_msg.z = input("Enter z position of point: ")
          grasp = input("Grasp? [y/n]")
          mover.follow_point(point_msg, grasp=True if grasp=='y' else False)

if __name__ == '__main__':
  main()
