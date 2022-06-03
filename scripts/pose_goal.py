#!/usr/bin/env python3

from demo_interface import DemoInterface
import math

def main():
  mover = DemoInterface()
  while True:
      r = float(input("Enter roll (x): "))
      p = float(input("Enter pitch (y): "))
      y = float(input("Enter yaw (z): "))
      rpy = [r,p,y]
      for num, val in enumerate(rpy):
          rpy[num] = math.radians(val)
      # The below methods are obsolete
      # mover.go_to_rpy_goal(rpy)
      # mover.go_to_pose_goal(rpy)

if __name__ == '__main__':
  main()
