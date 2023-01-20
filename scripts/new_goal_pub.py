#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64MultiArray

FINAL_JOINT_GOAL = [1.1901980109241104, 0.9615559746057705, -0.5881359185350531,
                    -1.2015471132200233, 0.5281574185640393, 2.0160775068768824,
                    1.3658315499054479]

if __name__ == "__main__":
    rospy.init_node("new_goal_publisher")
    new_goal_pub = rospy.Publisher("/new_planner_goal", Float64MultiArray, queue_size=1)
    rospy.sleep(0.1)
    new_goal_msg = Float64MultiArray()
    new_goal_msg.data = FINAL_JOINT_GOAL
    new_goal_pub.publish(new_goal_msg)
