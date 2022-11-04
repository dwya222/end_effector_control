#!/usr/bin/env python3

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from demo_interface import DemoInterface
import rospy
from geometry_msgs.msg import Point

def euler(pose):
    new_rot = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    output = np.array(new_rot.as_euler('xyz', degrees=True))
    return output

def getPosFromCam():
    point = rospy.wait_for_message("/point_command", Point)
    return {'x': point.x, 'y': point.y, 'z': point.z}

def main():
    # df = pd.DataFrame(columns=['pos_from_panda', 'pos_from_cam', 'rpy_from_panda', 'rpy_from_cam'])
    df = pd.DataFrame(columns=['pos_from_panda', 'pos_from_cam'])
    mover = DemoInterface()
    np.set_printoptions(suppress=True)
    key = ' '
    while key != 'q':
        key = input("Type enter to see next pose or q to quit\n")
        current_pose = mover.move_group.get_current_pose(end_effector_link="ee_april_tag")
        current_rpy = mover.move_group.get_current_rpy(end_effector_link="ee_april_tag")
        current_joint_values = mover.move_group.get_current_joint_values()
        print("joint state info:")
        print(current_joint_values)
        print("pose info:")
        print(current_pose)
        output = current_rpy
        print("Euler [x, y, z]:")
        print(output)
        pos_from_panda = {"x": current_pose.pose.position.x,
                          "y": current_pose.pose.position.y,
                          "z": current_pose.pose.position.z}
        # rpy_from_panda = {"r": current_rpy[0],
        #                   "p": current_rpy[1],
        #                   "y": current_rpy[2]}
        pos_from_cam = getPosFromCam()
        # rpy_from_cam = {}
        from_both_dict = pd.DataFrame([{'pos_from_panda': pos_from_panda,
                                       'pos_from_cam': pos_from_cam}])
        df = pd.concat([df, from_both_dict], ignore_index=True)
        print(f'df: \n{df}')

    df.to_csv('./position_data.csv')



if __name__ == '__main__':
    main()
