#!/usr/bin/env python3
from threading import RLock

import rospy
import actionlib
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from abstractnode import AbstractNode

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TwistStamped, Twist, Vector3
from move_base_mod.msg import TrajectoryControllerAction, TrajectoryControllerGoal
from nav_msgs.msg import Odometry

from engix_msgs.msg import LocalTrajectoryStamped, Localization, Route


class ControllerNode(AbstractNode):

    def initialization(self):
        self.__lock = RLock()
        self.__last_loc = None
        self.__last_trajectory = None
        self.localization_publisher = rospy.Publisher(
            '/localization', Localization, queue_size=10)
        self.control_publisher = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=10)

        # rospy.Subscriber('/local_trajectory_plan', LocalTrajectoryStamped, self.__route_callback)
        rospy.Subscriber('/route', Route, self.__route_task_callback)
        rospy.Subscriber('/laser_odom_to_init', Odometry,
                         self.__odometry_callback)

    def local_trajectory_callback(self, message: LocalTrajectoryStamped):
        with self.__lock:
            poses = list()
            for pose in message.route:
                poses.append(self.__Posed2DtoPoseStamped(pose))

            self.__last_trajectory = message

        # self.send_msg(poses)

    def __Posed2DtoPoseStamped(self, pose2d):
        pose = Pose()
        pose.position.x = pose2d.x
        pose.position.y = pose2d.y
        pose_or = quaternion_from_euler(0, 0, pose2d.theta)
        pose.orientation.x = pose_or[0]
        pose.orientation.y = pose_or[1]
        pose.orientation.z = pose_or[2]
        pose.orientation.w = pose_or[3]

        posestamped = PoseStamped()
        posestamped.pose = pose
        posestamped.header.stamp = rospy.get_rostime()

        return posestamped

    def __route_task_callback(self, message):
        with self.__lock:
            point_list = list()

            for point in message.route:
                point_list.append([point.x, point.y, point.speed])

            self.__last_trajectory = point_list

            if len(self.__last_trajectory) == 0:
                self.__last_trajectory = None

    def __odometry_callback(self, message: Odometry):
        with self.__lock:

            localization_msg = self.__odom_to_loc(message)
            self.localization_publisher.publish(localization_msg)
            self.__last_loc = localization_msg

    def __odom_to_loc(self, odometry: Odometry):
        localization = Localization()
        localization.header = odometry.header
        localization.pose = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        localization.yaw = euler[2]

        return localization

    def send_msg(self, msg: list):
        goal = TrajectoryControllerGoal()
        goal.poses = msg
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()

    def work(self):
        with self.__lock:
            if self.__last_loc is None or self.__last_trajectory is None:
                return

            current_pos = (self.__last_loc.pose.x,
                           self.__last_loc.pose.y, self.__last_loc.yaw)

            target_point = self.__get_target_point(current_pos)
            linear_velocity = self.__linear_velocity_control(
                current_pos, target_point)
            angular_velocity = self.__angular_velocity_control(
                current_pos, target_point)

            self.send_control(linear_velocity, angular_velocity)

    def send_control(self, linear_velocity, angular_velocity):
        print('Some Data: ')
        print(linear_velocity)
        print(angular_velocity)
        message = Twist()
        # message.header.stamp = rospy.get_rostime()
        message.linear = linear_velocity
        message.angular = angular_velocity
        self.control_publisher.publish(message)

    def __get_target_point(self, cur_pose):
        # poses = self.__last_trajectory.route
        path = np.array([p[:2] for p in self.__last_trajectory])
        cur_pose = np.array(cur_pose[:2])

        idx = (np.linalg.norm(path - cur_pose, axis=1)).argmin()
        path = np.array(self.__last_trajectory[idx:])

        if len(path) == 1:
            return path[0]

        distances = np.linalg.norm(path - path[0], axis=1)
        distances = np.abs(distances - 1.0)
        target_idx = distances.argmin()
        return path[target_idx]

    def __linear_velocity_control(self, current_pos, target_point):

        if np.linalg.norm(np.array(current_pos[:2]) - np.array(target_point[:2])) < 0.5:
            return Vector3(x=0.0, y=0.0, z=0.0)

        # if abs(current_pos[2] - target_point[2]) > 0.

        return Vector3(x=1.0, y=0.0, z=0.0)

    def __angular_velocity_control(self, current_pos, target_point):
        if abs(current_pos[2] - target_point[2]) < 0.1:
            return Vector3(x=0.0, y=0.0, z=0.0)

        rotate = 1.0 if current_pos[2] - target_point[2] > 0.0 else -1.0
        return Vector3(x=0.0, y=0.0, z=rotate)


def main():
    node = ControllerNode('ControllerNode', 10)
    node.run()


if __name__ == '__main__':
    main()
