import rospy
from geometry_msgs.msg import Pose
import geometry_msgs.msg
pose_publisher = rospy.Publisher('/pose_command', Pose, queue_size=10)
rospy.init_node('pose_pub')
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.7
pose_goal.orientation.x = 0.0
pose_goal.orientation.y = 0.0
pose_goal.orientation.z = 0.0
pose_goal.orientation.w = 1.0
pose_publisher.publish(pose_goal)
