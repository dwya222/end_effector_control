#!/usr/bin/env python3

from demo_interface import DemoInterface
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import rospy
import pandas as pd
import os.path
from os import path

def sample_env(origin, num_cups=12, iter=5):
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
    df = pd.DataFrame(columns=['id', 'front', 'left', 'right', 'back'])
    for i in range(0,num_cups):
        for k in range(0,iter):
            joint_trajectory_list = ['undetermined','undetermined','undetermined','undetermined']
            id = "cup_" + str(i+1) + "_" + str(k+1)
            result = [id, 'undetermined','undetermined','undetermined','undetermined']
            plan_msg = ['undetermined','undetermined','undetermined','undetermined']
            for j,approach in enumerate(approaches):
                joint_trajectory_list[j] = d.plan_to_point(cup_points[i], approach=approach)
                if joint_trajectory_list[j].joint_trajectory.points:
                    result[j+1] = "Success"
                else:
                    result[j+1] = "Fail"
            df.loc[(i*iter)+k] = result
    return(df)

def cup_summary(df, num_cups=12):
    iter = len(df)/num_cups
    df['cup_summary'] = '-----'
    for i in range(0, num_cups):
        cup_df = df[(iter*i):(iter*(i+1))]
        cup_summary = '%.1f%%' % (float(len(cup_df[cup_df['grasp_summary'] == 'Success'])) / len(cup_df) * 100)
        df['cup_summary'][i*iter] = cup_summary
    return df

def grasp_summary(row):
    if (row['front'] == 'Success' or row['left'] == 'Success' or
        row['right'] == 'Success' or row['back'] == 'Success'):
            return 'Success'
    else:
        return 'Fail'

if __name__ == "__main__":
    position = input("Which position? (1,2,3,4,5,6 or 7)\n")
    position_dict = {1: np.array([0.0, 0.2, -0.75]), 2: np.array([0.4, 0.5, -0.75]),
                     3: np.array([0.15, 0.5, -0.75]), 4: np.array([-0.4, 0.375, -0.75]),
                     5: np.array([-0.55, 0.75, -0.75]), 6: np.array([0.15, 1.0, -0.75]),
                     7: np.array([-0.4, 1.125, -0.75])}
    origin = position_dict[int(position)]
    num_cups=12
    iter=5

    # Sampling plans and creating df for successful grasp plans in environment
    df = sample_env(origin, num_cups=num_cups, iter=iter)
    print("sample df:")
    print(df)
    # Adding grasp summary row that checks if any of the 4 approaches were successful
    df['grasp_summary'] = df.apply(lambda row: grasp_summary(row), axis=1)
    print("grasp summary df:")
    print(df)
    # Adding cup summary row that shows which cups were able to be reached across all trials and grasps
    df = cup_summary(df, num_cups=num_cups)
    print("cup summary df:")
    print(df)
    # Creating totals row with percentage success and appending as last row of df
    amount = []
    for approach in ['front', 'left', 'right', 'back', 'grasp_summary']:
        amount.append(float((len(df[df[approach] == 'Success'])))/len(df) * 100)
    df.loc[len(df)] = ['totals', '%.1f%%' % amount[0], '%.1f%%' % amount[1], '%.1f%%' % amount[2],
                       '%.1f%%' % amount[3], '%.1f%%' % amount[4], '-----']
    print("totals df:")
    print(df)

    base_file_name = "position_" + str(position) + "_"
    # base_file_name = "origin_at_(" + str(origin[0]) + "," + str(origin[1]) + "," + str(origin[2]) + ")_"
    file_name = base_file_name
    version = 1
    while path.exists("/home/robotcontroller/environment_testing/" + file_name + ".csv"):
        file_name = base_file_name + str(version)
        version += 1

    df.to_csv('~/robo_demo_ws/src/end_effector_control/scripts/test_results/' + file_name + ".csv")
    df.to_pickle('~/robo_demo_ws/src/end_effector_control/scripts/test_results/' + file_name + ".pkl")
    # df_styled = summarized_df.style.background_gradient() #adding a gradient based on values in cell
    # dfi.export(df_styled,'first_attempt.png')
