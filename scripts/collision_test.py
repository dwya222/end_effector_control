#!/usr/bin/env python3

import rospy
from demo_interface import DemoInterface
from geometry_msgs.msg import Point
import threading as th

DEBUG = False

def requestPoint():
    if DEBUG:
        print("Where would you like to publish an obstacle?")
        x = input("x: ")
        y = input("y: ")
        z = input("z: ")
        r = input("radius: ")
    else:
        (x, y, z, r) = (0.5, -0.4, 0.6, 0.25)
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
    (point, radius) = requestPoint()

    # Add obstacle to scene
    rospy.sleep(1.0) # Need to sleep for a second to load up planning scene
    d.publish_object("obstacle", point, radius, type='sphere')

    # Start planning
    # d.follow_point(point)
    remove_obstacle_timer = th.Timer(3.0, remove_obstacle, args=(d,))
    rospy.loginfo("Starting thread to remove object in 3 seconds")
    remove_obstacle_timer.start()

    rospy.loginfo(f"Starting planning for {d.get_planning_time()} seconds")
    (success, plan, planning_time, error_code) = (d.planning_test(point))

    rospy.loginfo(f"Success status: {success}")


