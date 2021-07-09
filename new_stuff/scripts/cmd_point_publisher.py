import rospy
from geometry_msgs.msg import Point
import geometry_msgs.msg
point_publisher = rospy.Publisher('/point_command', Point, queue_size=10)
rospy.init_node('point_pub')
point_goal = geometry_msgs.msg.Point()
point_goal.x = 0.5
point_goal.y = 0.0
point_goal.z = 0.4
point_publisher.publish(point_goal)
