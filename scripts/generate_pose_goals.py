#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Point, PoseStamped

CAMERA_TOPIC = "/point_command"
TARGET_POSE_TOPIC = "/servo_server/target_pose"


class PointToPose():

    def __init__(self):
        self.camera_sub = rospy.Subscriber(CAMERA_TOPIC, Point, self.publish_pose)
        self.target_pub = rospy.Publisher(TARGET_POSE_TOPIC, PoseStamped, queue_size=1)

    def publish_pose(self, point):
        target_msg = PoseStamped()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = "panda_link0"
        target_msg.pose.position = point
        target_msg.pose.orientation.w = 1.0
        self.target_pub.publish(target_msg)


if __name__ == "__main__":
    rospy.init_node("generate_pose_goals")
    ptp = PointToPose()
    rospy.spin()
