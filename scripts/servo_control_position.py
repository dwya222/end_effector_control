#!/usr/bin/env python3

import numpy as np
from pyquaternion import Quaternion
from collections import deque

import rospy

from geometry_msgs.msg import TwistStamped, Point
from demo_interface import DemoInterface

DIFF_THRESHOLD = 0.01 # Meters
TWIST_THRESHOLD = 0.3
BUFFER_LENGTH = 3

INITIAL_POSITION = {'x': 0.30699960973309925,
                    'y': -0.0001944216291500493,
                    'z': 0.4401078414463817}

INITIAL_POSE = {'x': 0.9999999643736364,
                'y': 0.00019174948657225556,
                'z': -0.00014213554145435913,
                'w': 0.00011950877831704541}

INPUT_WORKSPACE_LIMIT_VAR = {'x_var': 0.2, 'y_var': 0.2, 'z_var': 0.2}
INPUT_WORKSPACE_LIMITS = {'x_min':  0.25, 'x_max': 0.60,
                          'y_min': -0.15, 'y_max': 0.15,
                          'z_min':  0.30, 'z_max': 0.50}
kp_position = 2.0
kp_pose = 0.5


class ServoPublisher():

    def __init__(self):
        self.initial_ee_position = INITIAL_POSITION
        self.desired_ee_pose = Quaternion(INITIAL_POSE['x'], INITIAL_POSE['y'],
                                          INITIAL_POSE['z'], INITIAL_POSE['w'])
        self.point_buffer = deque()
        self.initial_point = None
        self.interface = DemoInterface(True)
        self.delta_sub = rospy.Subscriber('/point_command', Point, self.delta_cb)
        self.twist_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped,
                                         queue_size=1)

    def delta_cb(self, delta):
        point = Point()
        # Convert deltas to position in robot frame
        point.x = self.initial_ee_position['x'] + delta.x
        point.y = self.initial_ee_position['y'] + delta.y
        point.z = self.initial_ee_position['z'] + delta.z
        self.point_buffer.append(point)
        if len(self.point_buffer) < BUFFER_LENGTH:
            return
        self.point_buffer.popleft()
        # Low pass filter point
        filtered_point = self.average_buffer()
        # Project point onto workspace limit box if it is outside
        projected_point = self.project_point(filtered_point)
        # Extract current end effector pose for FB control
        ee_pose = self.interface.move_group.get_current_pose(end_effector_link="panda_hand").pose
        ee_position = ee_pose.position
        ee_orientation = ee_pose.orientation
        # Compute position control
        position_diff = self.proportional_diff(ee_position, projected_point)
        vel_command = self.scale(position_diff)
        # Compute pose control
        # angular_vel_command = self.compute_pose_control(ee_orientation)
        # rospy.loginfo(f"angular_vel_command: {angular_vel_command}")

        # self.publish_delta_twist(vel_command, angular_vel_command)
        self.publish_delta_twist(vel_command, Point(0,0,0))

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

    def project_point(self, point):
        if point.x > INPUT_WORKSPACE_LIMITS['x_max']:
            point.x = INPUT_WORKSPACE_LIMITS['x_max']
        elif point.x < INPUT_WORKSPACE_LIMITS['x_min']:
            point.x = INPUT_WORKSPACE_LIMITS['x_min']
        if point.y > INPUT_WORKSPACE_LIMITS['y_max']:
            point.y = INPUT_WORKSPACE_LIMITS['y_max']
        elif point.y < INPUT_WORKSPACE_LIMITS['y_min']:
            point.y = INPUT_WORKSPACE_LIMITS['y_min']
        if point.z > INPUT_WORKSPACE_LIMITS['z_max']:
            point.z = INPUT_WORKSPACE_LIMITS['z_max']
        elif point.z < INPUT_WORKSPACE_LIMITS['z_min']:
            point.z = INPUT_WORKSPACE_LIMITS['z_min']
        return point

    def proportional_diff(self, ee_pos, target_pos):
        x_diff = ee_pos.x - target_pos.x
        y_diff = ee_pos.y - target_pos.y
        z_diff = ee_pos.z - target_pos.z
        diff_point = Point(kp_position * x_diff, kp_position * y_diff, kp_position * z_diff)
        return diff_point

    def scale(self, vec):
        mag = np.sqrt((vec.x ** 2) + (vec.y ** 2) + (vec.z ** 2))
        if mag > TWIST_THRESHOLD:
            scale = TWIST_THRESHOLD / mag
            vec.x = scale * vec.x
            vec.y = scale * vec.y
            vec.z = scale * vec.z
        return vec

    # def compute_pose_control(self, current_pose):
    #     current_quat = Quaternion(current_pose.x, current_pose.y, current_pose.z, current_pose.w)
    #     error = self.desired_ee_pose.conjugate * current_quat
    #     twist_cmd = np.array([error.vector * kp_pose])
    #     # return Point(twist_cmd[0][0], twist_cmd[0][1], -twist_cmd[0][2])
    #     return Point(twist_cmd[0][0], twist_cmd[0][1], 0)

    def compute_pose_control(self, current_pose):
        quat = Quaternion(current_pose.x, current_pose.y, current_pose.z, current_pose.w)
        quat_mat = np.array([[quat.z, quat.w, -quat.x],
                             [-quat.y, quat.x, quat.w],
                             [-quat.x, -quat.y, -quat.z]])
        twist_cmd = np.linalg.inv(quat_mat) @ np.array([-quat.y, -quat.z, -quat.w])
        return Point(twist_cmd[0], twist_cmd[1], twist_cmd[2])

    def publish_delta_twist(self, scaled_vector, angular_vel_command=Point(0,0,0)):
        ts = TwistStamped()
        ts.header.frame_id = "panda_link0"
        ts.header.stamp = rospy.Time.now()
        ts.twist.linear.x = scaled_vector.x
        ts.twist.linear.y = scaled_vector.y
        ts.twist.linear.z = scaled_vector.z
        ts.twist.angular.x = angular_vel_command.x
        ts.twist.angular.y = angular_vel_command.y
        ts.twist.angular.z = angular_vel_command.z
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
