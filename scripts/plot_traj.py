#!/usr/bin/env python3

from demo_interface import DemoInterface
from geometry_msgs.msg import Point, PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import rospy

def get_traj_lists(plan, joint_num):
    joint_positions = []
    joint_velocities = []
    joint_accelerations = []
    time_steps = []
    for traj_point in plan.joint_trajectory.points:
        joint_positions.append(traj_point.positions[joint_num])
        joint_velocities.append(traj_point.velocities[joint_num])
        joint_accelerations.append(traj_point.accelerations[joint_num])
        time_steps.append(traj_point.time_from_start.to_sec())
    return joint_positions, joint_velocities, joint_accelerations, time_steps

if __name__ == "__main__":
    d = DemoInterface()
    point = Point()
    point.x = 0.5
    point.y = 0.2
    point.z = 0.6
    # plan = d.planning_test(point)
    joint_nums = [1,2,3,4,5]
    # [plan1, plan2, start1, start2, last_point_plan1] = d.change_traj()
    [plan1, plan2, start1, start2] = d.change_traj()
    # rospy.loginfo("last_point_plan1: \n%s", last_point_plan1)
    for i in np.arange(0,len(joint_nums)):
        joint_num = joint_nums[i]
        start_diff = start2-start1
        [joint_pos1, joint_vel1, joint_acc1, t_steps1] = get_traj_lists(
                                                           plan1, joint_num)
        [joint_pos2, joint_vel2, joint_acc2, t_steps2] = get_traj_lists(
                                                           plan2, joint_num)
        for j in np.arange(0,len(t_steps2)):
            t_steps2[j] = t_steps2[j] + start_diff.to_sec()

        fig, ax = plt.subplots(3,1)
        ax[0].plot(t_steps1, joint_pos1, marker='.', label='Plan 1')
        ax[0].plot(t_steps2, joint_pos2, marker='.', label='Plan 2')
        ax[0].vlines(start_diff.to_sec(), min(joint_pos1),
                     max(joint_pos1),label='Plan 2 Execution')
        ax[0].set_title('Joint %s Position' %joint_num)
        ax[0].set_xlabel('Time (sec)')
        ax[0].set_ylabel('Position (rad)')
        ax[0].legend()
        ax[1].plot(t_steps1, joint_vel1, marker='.', label='Plan 1')
        ax[1].plot(t_steps2, joint_vel2, marker='.', label='Plan 2')
        ax[1].vlines(start_diff.to_sec(), min(joint_vel1),
                     max(joint_vel1), label='Plan 2 Execution')
        ax[1].set_title('Joint %s Velocity' %joint_num)
        ax[1].set_xlabel('Time (sec)')
        ax[1].set_ylabel('Velocity (rad/sec)')
        ax[1].legend()
        ax[2].plot(t_steps1, joint_acc1, marker='.', label='Plan 1')
        ax[2].plot(t_steps2, joint_acc2, marker='.', label='Plan 2')
        ax[2].vlines(start_diff.to_sec(), min(joint_acc1),
                     max(joint_acc1), label='Plan 2 Execution')
        ax[2].set_title('Joint %s Acceleration' %joint_num)
        ax[2].set_xlabel('Time (sec)')
        ax[2].set_ylabel('Acceleration (rad/sec^2)')
        ax[2].legend()
    plt.show()

