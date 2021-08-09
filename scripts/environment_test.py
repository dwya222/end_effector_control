#!/usr/bin/env python

from demo_interface import DemoInterface
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import rospy
import pandas as pd
import os.path
from os import path

def sample_env(origin):
    # origin is a list of 3 points taken from the URDF file for the origin of the stl mesh
    # define cup [x,y,z] offsets and subtract them from the origin
    cup_locations = np.empty((12,3))
    cup_locations[0] = np.array(origin - np.array([-0.08, 0.4625, -0.825]))
    cup_locations[1] = np.array(origin - np.array([-0.08, 1.06, -0.825]))
    cup_locations[2] = np.array(origin - np.array([-0.28, 0.631, -0.825]))
    cup_locations[3] = np.array(origin - np.array([-0.28, 0.8915, -0.825]))
    cup_locations[4] = np.array(origin - np.array([-0.6825, 0.4625, -0.825]))
    cup_locations[5] = np.array(origin - np.array([-0.6825, 1.06, -0.825]))

    cup_locations[6] = np.array(cup_locations[0] - [0, 0, -0.31])
    cup_locations[7] = np.array(cup_locations[1] - [0, 0, -0.31])
    cup_locations[8] = np.array(cup_locations[2] - [0, 0, -0.31])
    cup_locations[9] = np.array(cup_locations[3] - [0, 0, -0.31])
    cup_locations[10] = np.array(cup_locations[4] - [0, 0, -0.31])
    cup_locations[11] = np.array(cup_locations[5] - [0, 0, -0.31])

    cup_points = []
    for loc in cup_locations:
        point = Point()
        point.x = loc[0]
        point.y = loc[1]
        point.z = loc[2]
        cup_points.append(point)

    d = DemoInterface()
    d.display_points(cup_points)

    approaches = ['front', 'left', 'right', 'back']
    success_df = pd.DataFrame(columns=['front', 'left', 'right', 'center'])
    for i in range(0,12):
        result = ['undetermined','undetermined','undetermined','undetermined']
        for j,approach in enumerate(approaches):
            (result[j], plan_msg, time, error_code) = d.planning_test(cup_points[i], approach=approach)
        success_df.loc[i] = result

    return(success_df)

if __name__ == "__main__":
    origin = np.array([-0.55, 0.75, -0.75])
    success_df = sample_env(origin)
    file_name = "origin_at_(" + str(origin[0]) + "," + str(origin[1]) + "," + str(origin[2]) + ")_"

    version = 1
    while path.exists("/home/robotcontroller/environment_testing/" + file_name + ".csv"):
        file_name = file_name + str(version)
        version += 1

    success_df.to_csv("/home/robotcontroller/environment_testing/" + file_name + ".csv")
