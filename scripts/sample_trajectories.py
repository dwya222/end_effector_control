#!/usr/bin/env python

"""
Need to figure out how to turn off rviz.
"""
from rospy_message_converter import json_message_converter
from demo_interface import DemoInterface
from geometry_msgs.msg import Point
import json

if __name__ == "__main__":
    d = DemoInterface()
    # will generate points later but for now start with one point
    point = Point()
    point.x = 0.5
    point.y = 0.2
    point.z = 0.6

    # next plan to point
    plan = d.planning_test(point, approach="follow")
    
    # now save the plan as pickle/json
    json_plan = json_message_converter.convert_ros_message_to_json(plan)
    with open('./trajectory_samples/plan.json', 'w') as f:
        json.dump(json_plan, f)

    # next: - look into converting to a dictionary first
    #       - look into saving as a pickle
    #       - look into loading and parsing the files
