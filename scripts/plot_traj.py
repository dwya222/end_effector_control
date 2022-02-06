#!/usr/bin/env python

from demo_interface import DemoInterface
from geometry_msgs.msg import Point, PoseStamped
import matplotlib.pyplot as plt
import numpy as np

def get_traj_lists(plan, joint_num):
    joint_positions = []
    joint_velocities = []
    time_steps = []
    for traj_point in plan.joint_trajectory.points:
        joint_positions.append(traj_point.positions[joint_num])
        joint_velocities.append(traj_point.velocities[joint_num])
        time_steps.append(traj_point.time_from_start.to_sec())
    return joint_positions, joint_velocities, time_steps

def plot_traj(joint_num, plan=None, joint_pos=None,
              joint_vel=None, t_steps=None):
    if plan is not None:
        [joint_pos, joint_vel, t_steps] = get_traj_lists(plan, joint_num)

    ax[0].plot(t_steps, joint_pos)
    ax[0].set_title('Joint %s Position' %joint_num)
    ax[0].set_xlabel('Time (sec)')
    ax[0].set_ylabel('Position (rad)')
    ax[1].plot(t_steps, joint_vel)
    ax[1].set_title('Joint %s Velocity' %joint_num)
    ax[1].set_xlabel('Time (sec)')
    ax[1].set_ylabel('Velocity (rad/sec)')

if __name__ == "__main__":
    d = DemoInterface()
    point = Point()
    point.x = 0.5
    point.y = 0.2
    point.z = 0.6
    # plan = d.planning_test(point)
    joint_num = 3
    [plan1, plan2, start1, start2] = d.change_traj()
    start_diff = start2-start1
    [joint_pos1, joint_vel1, t_steps1] = get_traj_lists(plan1, joint_num)
    [joint_pos2, joint_vel2, t_steps2] = get_traj_lists(plan2, joint_num)
    for i in np.arange(0,len(t_steps2)):
        t_steps2[i] = t_steps2[i] + start_diff.to_sec()

    fig, ax = plt.subplots(1,2)
    ax[0].plot(t_steps1, joint_pos1, label='Plan 1')
    ax[0].plot(t_steps2, joint_pos2, label='Plan 2')
    ax[0].set_title('Joint %s Position' %joint_num)
    ax[0].set_xlabel('Time (sec)')
    ax[0].set_ylabel('Position (rad)')
    ax[0].legend()
    ax[1].plot(t_steps1, joint_vel1, label='Plan 1')
    ax[1].plot(t_steps2, joint_vel2, label='Plan 2')
    ax[1].set_title('Joint %s Velocity' %joint_num)
    ax[1].set_xlabel('Time (sec)')
    ax[1].set_ylabel('Velocity (rad/sec)')
    ax[1].legend()

    # plot_traj(joint_num, joint_pos=joint_pos1, joint_vel=joint_vel1,
    #           t_steps=t_steps1)
    # plot_traj(joint_num, joint_pos=joint_pos1, joint_vel=joint_vel1,
    #           t_steps=t_steps1)
    plt.show()
