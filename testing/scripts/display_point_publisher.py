#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

class point_pub(object):
    def __init__(self, node_init=False):
        self.pub = rospy.Publisher('/rviz_visual_tools', MarkerArray, queue_size = 100)
        if node_init:
            rospy.init_node('point_publishing_node')
        rospy.sleep(1)

    def display_point(self, point):
        markerArray = MarkerArray()
        count = 0
        MARKERS_MAX = 1
        while count == 0:
            marker = Marker()
            marker.header.frame_id = "panda_link0"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
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
    point_pub = point(node_init=True)
    point_pub.display_point([.6,0,.4])
