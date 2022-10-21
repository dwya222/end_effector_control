#!/usr/bin/env python3

import rospy
from demo_interface import DemoInterface
from geometry_msgs.msg import Point
import threading as th

DEBUG = False

def requestPoint():
    if DEBUG:
        print("Where would you like to publish an obstacle?")
        x = float(input("x: "))
        y = float(input("y: "))
        z = float(input("z: "))
        r = float(input("radius: "))
    else:
        (x, y, z, r) = (0.5, -0.4, 0.6, 0.25)
    return (pointMsg(x, y, z), r)

def pointMsg(x, y, z):
    msg = Point()
    msg.x = x
    msg.y = y
    msg.z = z
    return msg

def add_obstacle(demo_interface_object, point, radius):
    rospy.loginfo("Adding obstacle now")
    d.publish_object("obstacle", point, radius, type='sphere')

def remove_obstacle(demo_interface_object):
    rospy.loginfo("Removing obstacle now")
    demo_interface_object.remove_object("obstacle")


if __name__ == "__main__":
    d = DemoInterface()
    (point, radius) = requestPoint()

    # Add obstacle to scene
    rospy.sleep(1.0) # Need to sleep for a second to load up planning scene
    # add_obstacle(d, point, radius)
    remove_obstacle(d)

    # Start threads before planning
    delay = 1.0
    # remove_obstacle_timer = th.Timer(delay, remove_obstacle, args=(d,))
    add_obstacle_timer = th.Timer(delay, add_obstacle, args=(d, point, radius))
    # rospy.loginfo(f"Starting thread to remove obstacle in {delay} seconds")
    # remove_obstacle_timer.start()
    rospy.loginfo(f"Starting thread to add obstacle back in {delay} seconds")
    add_obstacle_timer.start()

    rospy.loginfo(f"Starting planning for {d.get_planning_time()} seconds")
    (success, plan, planning_time, error_code) = (d.planning_test(point))

    rospy.loginfo(f"Success status: {success}")


