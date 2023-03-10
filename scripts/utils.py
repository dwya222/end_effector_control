import numpy as np

from trajectory_msgs.msg import JointTrajectoryPoint

def path_cost(path, norm_ord=1, steps=False):
    """
    args:
        path (list of JointTrajectoryPoints OR list of lists)
    """
    step_costs = []
    if isinstance(path[0], JointTrajectoryPoint):
        for i in range(len(path) - 1):
            step_costs.append(step_cost(path[i].positions, path[i+1].positions, norm_ord=norm_ord))
    elif isinstance(path[0], list) or isinstance(path[0], tuple):
        for i in range(len(path) - 1):
            step_costs.append(step_cost(path[i], path[i+1], norm_ord=norm_ord))
    else:
        print(f"Not computing costs: type: {type(path[0])}")
    cost = sum(step_costs)
    if steps:
        return (cost, step_costs)
    return cost

def step_cost(vec1, vec2, norm_ord=1):
    """
    args:
        vec1 (list)
        vec2 (list)
    """
    position1 = np.array(vec1)
    position2 = np.array(vec2)
    return np.linalg.norm(position2 - position1, ord=norm_ord)

def almost_equal(vec1, vec2, tol=0.01):
    """
    args:
        vec1 (list)
        vec2 (list)
    """
    if len(vec1) != len(vec2):
        return False
    for (val1, val2) in zip(vec1, vec2):
        if abs(val1 - val2) > tol:
            return False
    return True

