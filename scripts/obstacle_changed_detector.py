#!/usr/bin/env python3

"""

This ObstaclesChangedDetector node subscribes to
to the move_group/monitored_planning_scene topic and checks to
see if any new obstacles have been added to the scene or if any old obstacles have moved. If either
of these cases are true, it will publish true to the "/obstacles_changed" topic

"""
"""
see if the position/primitives.dimensions of
world.collision_objects.id = "obstacle*"s have changed
If they have changed then publish true to the obstacles_changed
topic (subscribed to by callback below"
Otherwise don't publish anything (since there will be many
messages coming across the topic during this process due to the
moving visualization of the robot arm
"""

import rospy

from moveit_msgs.msg import PlanningScene
from std_msgs.msg import Bool


class ObstaclesChangedDetector():

    def __init__(self):
        self.init_sub_pub()
        self.obstacles_dict = {}

    def init_sub_pub(self):
        rospy.loginfo("Initializing subscriber and publisher")
        self.planning_scene_sub = rospy.Subscriber("/move_group/monitored_planning_scene",
                                                   PlanningScene, self.planning_scene_cb,
                                                   queue_size=10)
        self.obstacles_changed_pub = rospy.Publisher("/obstacles_changed", Bool, queue_size=1)

    def planning_scene_cb(self, planning_scene_msg):
        obstacles = planning_scene_msg.world.collision_objects
        changed = False
        for obstacle in obstacles:
            if obstacle.id in self.obstacles_dict.keys():
                # We have seen this obstacle before, check if it has
                # changed position or size
                # TODO: may want to check if the primitives
                # (shape/dimension) changes too in the future
                # TODO: don't need this to be EXACTLY equal, just nearby
                if self.obstacles_dict[obstacle.id]["position"] != obstacle.pose.position:
                    rospy.loginfo("old obstacle position changed")
                    rospy.loginfo(f"obstacle.id: {obstacle.id}\nposition: {obstacle.pose.position}")
                    # only want to publish at most once per
                    # PlanningScene msg so end for loop now
                    changed = True
                    self.obstacles_dict[obstacle.id] = {"position": obstacle.pose.position}
                else:
                    rospy.loginfo("old obstacle position did not change")
                    rospy.loginfo(f"obstacle.id: {obstacle.id}\nposition: {obstacle.pose.position}")
            else:
                rospy.loginfo("new obstacle")
                rospy.loginfo(f"obstacle.id: {obstacle.id}\nposition: {obstacle.pose.position}")
                # add to previously seen obstacles
                self.obstacles_dict[obstacle.id] = {"position": obstacle.pose.position}
                changed = True
            ## TODO what if an obstacle gets removed and then added back in the same exact place?
        if changed == True:
            self.obstacles_changed_pub.publish(True)


if __name__== "__main__":
    rospy.init_node("obstacles_changed_detector")
    ocd = ObstaclesChangedDetector()
    rospy.loginfo("Detector started, waiting for messages from /move_group/monitored_planning_scene")
    rospy.spin()

