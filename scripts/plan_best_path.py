#!/usr/bin/env python3

"""

Script that takes in RTRRTstar run data and runs RRTstar for a 'long
time' (i.e. 10 sec) on same start/goal to get output for cost
comparison.

Usage:
    `python3 plan_best_path.py [RTRRT_json_file_path]`

"""

import json
import sys

from demo_interface import DemoInterface

def retrieve_json():
    if (len(sys.argv) != 2):
        raise ValueError(f"Expected sys.argv length 2, recieved sys.argv length {len(sys.argv)}")
    with open(sys.argv[1], 'r') as f:
        RTRRTjson = json.load(f)
    return RTRRTjson

def get_start_and_goal_states(json):
    num_goals = len(json['Goals'])
    start_state = json['Goal_' + str(num_goals - 1) + '_states'][0]
    last_goal_state = json['Goals'][-1]

    return (start_state, last_goal_state)

if __name__ == "__main__":
    d = DemoInterface()
    d.move_group.set_planner_id("RRTstarkConfigDefault")
    RTRRTjson = retrieve_json()
    (start, goal) = get_start_and_goal_states(RTRRTjson)
    d.go_to_joint_goal(start)
    d.set_planning_time(10.0)
    d.plan_to_joint_goal(goal)

