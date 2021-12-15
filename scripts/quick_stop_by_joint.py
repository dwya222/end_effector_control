#!/usr/bin/env python

from demo_interface import DemoInterface
import rospy
import numpy as np

d = DemoInterface()
d.go_to_start()
joint_vals = d.move_group.get_current_joint_values()

for i in np.arange(0,len(joint_vals)):
    joint_vals = d.move_group.get_current_joint_values()
    joint_vals[i] = joint_vals[i] + 0.5
    d.move_group.set_joint_value_target(joint_vals)
    plan = d.move_group.plan()
    d.move_group.execute(plan,wait=False)
    rospy.sleep(1)
    d.move_group.stop()
