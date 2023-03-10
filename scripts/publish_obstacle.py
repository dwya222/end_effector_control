#!/usr/bin/env python3

import rospy
from demo_interface import DemoInterface
from geometry_msgs.msg import Point
import threading as th


def requestPoint(default=False):
    if default:
        # (x, y, z, r) = (0.5, -0.4, 0.6, 0.05)
        # (x, y, z, r) = (0.3, -0.3, 0.3, 0.05)
        # (x, y, z, r) = (0.5, 0.0, 0.6, 0.05)
        # (x, y, z, r) = (0.4, -0.3, 0.4, 0.05)
        (x, y, z, r) = (0.3, -0.4, 0.6, 0.05)
    else:
        rospy.loginfo("Where would you like to publish an obstacle?")
        x = float(input("x: "))
        y = float(input("y: "))
        z = float(input("z: "))
        r = float(input("radius: "))
        rospy.loginfo(f"publishing obstacle at default location {x, y, z} with radius {r}")
    return (pointMsg(x, y, z), r)

def pointMsg(x, y, z):
    msg = Point()
    msg.x = x
    msg.y = y
    msg.z = z
    return msg

def remove_obstacle(demo_interface_object):
    rospy.loginfo("Removing obstacle now")
    demo_interface_object.remove_object("obstacle")

if __name__ == "__main__":
    d = DemoInterface()
    rospy.sleep(0.5) # Need to sleep for a second to load up planning scene

    # Add obstacle to scene
    default = 'd' == input("Input 'd' to use default point: ")
    while not rospy.is_shutdown():
        response = input("Add or remove obstacle ('a' or 'r' or 'ra')\n")
        if response != 'a' and response != 'r' and response != 'ra':
            rospy.logwarn("Invalid response")
            continue
        if response == 'a':
            (point, radius) = requestPoint(default)
            d.publish_object("obstacle", point, radius, primitive='sphere')
        if response == 'r':
            remove_obstacle(d)
        if response == 'ra':
            (point, radius) = requestPoint(default)
            remove_obstacle(d)
            d.publish_object("obstacle", point, radius, primitive='sphere')
