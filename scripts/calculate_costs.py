#!/usr/bin/env python3

"""

Usage:
    `python3 calculate_costs.py [RRT_json_file_path] [RTRRT_json_file_path]`

Need to make sure that the RRT run has (approximately) the same start
and goal states so that this cost comparison is meaningful.

"""

import json
import sys
import numpy as np

def retrieve_jsons():
    if (len(sys.argv) != 3):
        raise ValueError(f"Expected sys.argv length 3, recieved sys.argv length {len(sys.argv)}")
    with open(sys.argv[1], 'r') as f:
        RRTjson = json.load(f)
    with open(sys.argv[2], 'r') as f:
        RTRRTjson = json.load(f)
    return(RRTjson, RTRRTjson)

def calculate_cost(states):
    cost = 0
    for i in range(len(states) - 1):
        s1 = states[i]
        s2 = states[i + 1]
        for j in range(len(states[i])):
                diff = s1[j] - s2[j]
                cost += diff ** 2
    return np.sqrt(cost)

def get_last_goal_states(json):
    num_goals = len(json.items()) - 1
    last_goal_key = "Goal" + str(num_goals) + "States"
    return json[last_goal_key]

if __name__ == "__main__":
    (RRTjson, RTRRTjson) = retrieve_jsons()
    best_cost = calculate_cost(RRTjson['States'])

    RTRRT_states = get_last_goal_states(RTRRTjson)
    RTRRT_cost = calculate_cost(RTRRT_states)
    print(f"RRT start:\n{RRTjson['States'][0]}")
    print(f"RRT end:\n{RRTjson['States'][-1]}")
    print(f"RTRRT start:\n{RTRRT_states[0]}")
    print(f"RTRRT end:\n{RTRRT_states[-1]}")
    print(f"best cost: {best_cost}")
    print(f"RTRRT cost: {RTRRT_cost}")

