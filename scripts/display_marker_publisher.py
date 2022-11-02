#!/usr/bin/env python3

import roslib; #roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import Point

class marker_pub(object):
    def __init__(self, node_init=False):
        self.pub = rospy.Publisher('/rviz_visual_tools', MarkerArray, queue_size = 100)
        if node_init:
            rospy.init_node('marker_publishing_node')
        rospy.sleep(1)

    def display_marker(self, point_list):
        markerArray = MarkerArray()
        count = 0
        MARKERS_MAX = 15
        for point in point_list:
            marker = Marker()
            marker.header.frame_id = "panda_link0"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.b = 1.0
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = point.z
            marker.pose.orientation.w = 1.0

            # We add the new marker to the MarkerArray, removing the oldest
            # marker from it when necessary
            if(count > MARKERS_MAX):
                markerArray.markers.pop(0)

            markerArray.markers.append(marker)

            # Renumber the marker IDs
            id = 0
            for m in markerArray.markers:
                m.id = id
                id += 1

            # Publish the MarkerArray
            self.pub.publish(markerArray)
            count += 1
            rospy.sleep(0.01)



if __name__=='__main__':
    marker_pub = marker_pub(node_init=True)
    point1 = Point()
    point2 = Point()
    point1.x = .6
    point1.y = 0
    point1.z = .4
    point2.x = .5
    point2.y = .5
    point2.z = .5
    marker_pub.display_marker([point1,point2])
