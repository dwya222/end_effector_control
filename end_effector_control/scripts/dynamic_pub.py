import rospy
from geometry_msgs.msg import Point
import geometry_msgs.msg
point_publisher = rospy.Publisher('/point_command', Point, queue_size=1)
rospy.init_node('point_pub', anonymous=True)

point_goal = geometry_msgs.msg.Point()
point_goal.x = 0.7
point_goal.y = 0.0
point_goal.z = 0.3

rate = rospy.Rate(.25)

point_publisher.publish(point_goal)
rate.sleep()
point_publisher.publish(point_goal)
rate.sleep()
point_publisher.publish(point_goal)
rate.sleep()
point_publisher.publish(point_goal)
rate.sleep()
point_publisher.publish(point_goal)


point_goal.x = 0.7
point_goal.y = 0.2
point_goal.z = 0.8

point_publisher.publish(point_goal)
rate.sleep()
point_publisher.publish(point_goal)
rate.sleep()
point_publisher.publish(point_goal)
rate.sleep()
point_publisher.publish(point_goal)
rate.sleep()
point_publisher.publish(point_goal)
rate.sleep()
