#!/usr/bin/env python

from demo_interface import DemoInterface
import math

def main():
  mover = DemoInterface()
  while True:
      r = input("Enter roll (x): ")
      p = input("Enter pitch (y): ")
      y = input("Enter yaw (z): ")
      rpy = [r,p,y]
      for num, val in enumerate(rpy):
          rpy[num] = math.radians(val)
      # mover.go_to_rpy_goal(rpy)
      mover.go_to_pose_goal(rpy)

if __name__ == '__main__':
  main()
