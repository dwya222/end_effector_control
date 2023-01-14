#!/usr/bin/env python3

import rospy

from demo_interface import DemoInterface
from geometry_msgs.msg import Point

(PX, PY, PZ) = (0.5, 0.2, 0.3)

if __name__ == "__main__":
    d = DemoInterface()
    p = Point(PX, PY, PZ)

    while not rospy.is_shutdown():
        query = int(input("Enter 1 to plan and go to start or 2 to plan and go to set state: "))
        if query == 1:
            d.go_to_start()
        if query == 2:
            plan = d.planning_test(p)
            d.move_group.execute(plan[1], wait=False)
