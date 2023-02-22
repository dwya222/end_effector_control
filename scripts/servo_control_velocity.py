#!/usr/bin/env python3

import numpy as np
from collections import deque

import rospy

from geometry_msgs.msg import TwistStamped, Point
from demo_interface import DemoInterface

DIFF_THRESHOLD = 0.01 # Meters
TWIST_THRESHOLD = 0.15
WORKSPACE_LIMITS = {'x_min':  0.3, 'x_max': 0.4,
                    'y_min': -0.1, 'y_max': 0.1,
                    'z_min':  0.4, 'z_max': 0.5}


class ServoPublisher():

    def __init__(self):
        self.point_sub = rospy.Subscriber('/point_command', Point, self.point_cb)
        self.twist_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped,
                                         queue_size=1)
        self.demo_interface = DemoInterface(True)
        self.point_buffer = deque()
        self.set_point = None

    def point_cb(self, point):
        self.point_buffer.append(point)
        if len(self.point_buffer) < 6:
            return
        elif self.set_point == None:
            self.set_point = self.average_buffer()
        self.point_buffer.popleft()
        filtered_point = self.average_buffer()
        (diff, diff_vec) = self.threshold_diff(self.set_point, filtered_point)
        if diff:
            # Need to invert the point values to transform to world frame
            inverted_vec = self.invert(diff_vec)
            # Scale vector to limit max velocity command
            scaled_vec = self.scale(inverted_vec)
            # Restrict vector to send velocity commands within bounds
            restricted_vec = self.restrict(scaled_vec)
            self.publish_delta_twist(restricted_vec)

    def average_buffer(self):
        out_pt = Point()
        for point in self.point_buffer:
            out_pt.x += point.x
            out_pt.y += point.y
            out_pt.z += point.z
        out_pt.x /= len(self.point_buffer)
        out_pt.y /= len(self.point_buffer)
        out_pt.z /= len(self.point_buffer)
        return out_pt

    def threshold_diff(self, set_point, point):
        """ Return diff Point if above threshold, otherwise return zero Point. """
        diff_point = Point()
        x_diff = point.x - set_point.x
        y_diff = point.y - set_point.y
        z_diff = point.z - set_point.z

        diff = False
        if abs(x_diff) > DIFF_THRESHOLD:
            diff = True
            diff_point.x = x_diff
        if abs(y_diff) > DIFF_THRESHOLD:
            diff = True
            diff_point.y = y_diff
        if abs(z_diff) > DIFF_THRESHOLD:
            diff = True
            diff_point.z = z_diff
        return (diff, diff_point)

    def invert(self, point):
        point.x = -point.x
        point.y = -point.y
        point.z = -point.z
        return point

    def scale(self, vec):
        mag = np.sqrt((vec.x ** 2) + (vec.y ** 2) + (vec.z ** 2))
        if mag > TWIST_THRESHOLD:
            scale = TWIST_THRESHOLD / mag
            vec.x = scale * vec.x
            vec.y = scale * vec.y
            vec.z = scale * vec.z
        return vec

    def restrict(self, vec):
        current_pose = self.demo_interface.move_group.get_current_pose(end_effector_link="panda_hand")
        current_position = current_pose.pose.position
        if current_position.x > WORKSPACE_LIMITS['x_max'] and vec.x < 0:
            vec.x = 0
        elif current_position.x < WORKSPACE_LIMITS['x_min'] and vec.x > 0:
            vec.x = 0
        if current_position.y > WORKSPACE_LIMITS['y_max'] and vec.y < 0:
            vec.y = 0
        elif current_position.y < WORKSPACE_LIMITS['y_min'] and vec.y > 0:
            vec.y = 0
        if current_position.z > WORKSPACE_LIMITS['z_max'] and vec.z < 0:
            vec.z = 0
        elif current_position.z < WORKSPACE_LIMITS['z_min'] and vec.z > 0:
            vec.z = 0
        return vec

    def publish_delta_twist(self, scaled_vector):
        ts = TwistStamped()
        ts.header.frame_id = "panda_link0"
        ts.header.stamp = rospy.Time.now()
        ts.twist.linear.x = scaled_vector.x
        ts.twist.linear.y = scaled_vector.y
        ts.twist.linear.z = scaled_vector.z
        self.twist_pub.publish(ts)

    def publish_delta_twist_test(self):
        ts = TwistStamped()
        ts.header.frame_id = "panda_link0"
        ts.header.stamp = rospy.Time.now()
        ts.twist.linear.x = -0.15
        self.twist_pub.publish(ts)


if __name__ == "__main__":
    rospy.init_node('delta_twist_publisher')

    sp = ServoPublisher()
    rospy.spin()
