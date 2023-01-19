#!/usr/bin/env python3

import sys

import rospy
from demo_interface import DemoInterface
from geometry_msgs.msg import Point

DEBUG = True

if __name__ == "__main__":
    d = DemoInterface()
    if sys.argv[0] == 'rosrun' and len(sys.argv) > 2:
        point_topics = sys.argv[3:]
        rospy.loginfo(f"Showing points for topics {point_topics}")
    elif len(sys.argv) > 1:
        point_topics = sys.argv[1:]
        rospy.loginfo(f"Showing points for topics {point_topics}")
    else:
        rospy.loginfo("No point topics specified. Defaulting to /cal_ee_position")
        point_topics = ['/cal_ee_position']

    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        i = 0
        points = []
        for topic in point_topics:
            points.append(rospy.wait_for_message(topic, Point))
            d.publish_object(f"show_point{i}", point, 0.015, primitive='sphere')
            i += 1
