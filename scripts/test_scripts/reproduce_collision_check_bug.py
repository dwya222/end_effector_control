#!/usr/bin/env python3

import rospy

from robo_demo_msgs.msg import JointTrajectoryPointStamped, JointTrajectoryPointClearStamped
from moveit_msgs.msg import PlanningScene
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from demo_interface import DemoInterface

JOINT_STATE1 = [0.00, -0.785, 0.00, -2.356, 0.00, 1.571, 0.785]
JOINT_STATE2 = [0.46970780954230257, -1.2628916426133203, -0.02999052995269058,
                -1.1781099656834608, 0.11842522205050404, 2.6668986253326805, 0.3062932335248344]
JOINT_STATE3 = [0.6974779786902082, -1.1247930854123802, -0.9410489571388111, -1.9846089917922924,
                -0.01871075594262786, 3.102168489791953, -0.037874744378963154]
JOINT_STATE4 = [0.9023001111533858, -1.0006080743470505, -1.7603174792323928, -2.709852560671671,
                -0.1420301684673516, 3.493584548220631, -0.3473675081746465]
JOINT_STATES = [JOINT_STATE1, JOINT_STATE2, JOINT_STATE3, JOINT_STATE4]

class CommunicationChecker:

    def __init__(self):
        self.controller_sub = rospy.Subscriber('/executing_to_state', JointTrajectoryPointStamped,
                                               self.controller_cb)
        # self.collision_checker_sub = rospy.Subscriber('/edge_clear',
        #                                               JointTrajectoryPointClearStamped,
        #                                               self.collision_checker_cb)
        self.planning_scene_sub = rospy.Subscriber('/move_group/monitored_planning_scene',
                                                   PlanningScene, self.planning_scene_cb)
        self.current_path_pub = rospy.Publisher('/current_path', JointTrajectory, queue_size=1)
        self.demo_interface = DemoInterface(node_initialized=True)
        self.heading_to_end_position = False
        self.end_position = None
        self.current_path = self.get_current_path()
        rospy.sleep(0.5)
        rospy.loginfo("publishing current path")
        self.current_path_pub.publish(self.current_path)

    def get_current_path(self):
        joint_trajectory = JointTrajectory()
        for joint_state in JOINT_STATES:
            joint_state_msg = JointTrajectoryPoint()
            for i in range(len(joint_state)):
                joint_state_msg.positions.append(joint_state[i])
            joint_trajectory.points.append(joint_state_msg)
        return joint_trajectory

    def controller_cb(self, next_state):
        rospy.loginfo('in controller_cb')
        if list(next_state.trajectory_point.positions) == self.current_path.points[-1].positions:
            rospy.loginfo("Headed to last state in path")
            self.heading_to_end_position = True
        elif list(next_state.trajectory_point.positions) == self.current_path.points[-2].positions:
            rospy.loginfo("Headed to 2nd to last state in path, publishing obstacle")
            (x, y, z, r) = (0.4, -0.3, 0.4, 0.05)
            self.demo_interface.publish_object_xyz("obstacle", x, y, z, r, primitive='sphere', remove=False)
        else:
            rospy.loginfo(f"next_state:\n{next_state.trajectory_point.positions}\ncurrent_path:\n{self.current_path.points[-1].positions}")

    def planning_scene_cb(self, planning_scene):
        if planning_scene.world.collision_objects:
            if self.heading_to_end_position:
                rospy.loginfo("recieved obstacle but control heading to end position already: SHOULD NOT CATCH")
            else:
                rospy.loginfo("recieved obstacle and not heading to end position: SHOULD CATCH")


if __name__ == "__main__":
    rospy.init_node('communication_checker')
    cc = CommunicationChecker()
    rospy.spin()
