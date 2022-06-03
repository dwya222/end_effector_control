#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R
from demo_interface import DemoInterface

def euler(pose):
    new_rot = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    output = np.array(new_rot.as_euler('xyz', degrees=True))
    return output

def main():
  mover = DemoInterface()
  np.set_printoptions(suppress=True)
  while True:
       input("Type enter to see next pose \n")
       current_pose = mover.move_group.get_current_pose()
       current_joint_values = mover.move_group.get_current_joint_values()
       print("joint state info:")
       print(current_joint_values)
       print("pose info:")
       print(current_pose)
       output = euler(current_pose.pose)
       print("Euler [x, y, z]:")
       print(output)


if __name__ == '__main__':
  main()
