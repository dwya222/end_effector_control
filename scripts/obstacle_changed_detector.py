#!/usr/bin/env python3

"""

This ObstaclesChangedDetector node subscribes to
to the move_group/monitored_planning_scene topic and checks to
see if any new obstacles have been added to the scene or if any old obstacles have moved. If either
of these cases are true, it will publish true to the "/obstacles_changed" topic
TODO:
    - check if obstacles dimensions or orientation changed (just checking position now)
    - add in noise tolerance so this only publishes if the obstacle moves an amount above a certain
      threshold
    - make sure that we don't publish anything when we update (quickly remove and add) an object in
      the same (or close to the same) spot in the planning scene.
    - remove an obstacle from the already seen obstacles_dict if is no longer in the planning scene
      after a few messages so that if we remove an obstacle and then add it back in the same place
      some time later then it should be considered new.

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
                if self.obstacles_dict[obstacle.id]["position"] != obstacle.pose.position:
                    rospy.loginfo("old obstacle position changed")
                    rospy.loginfo(f"obstacle.id: {obstacle.id}\nposition: {obstacle.pose.position}")
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
        if changed == True:
            self.obstacles_changed_pub.publish(True)


if __name__== "__main__":
    rospy.init_node("obstacles_changed_detector")
    ocd = ObstaclesChangedDetector()
    rospy.loginfo("Detector started, waiting for messages from /move_group/monitored_planning_scene")
    rospy.spin()

