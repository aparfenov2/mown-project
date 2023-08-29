#!/usr/bin/env python3

import numpy as np
import rospy
import ros_numpy
from tf import TransformListener
from tf.transformations import quaternion_from_euler
import threading

from geometry_msgs.msg import Point, PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from abstractnode import AbstractNode
from engix_msgs.msg import RouteTaskToPoint, Route, Localization
from enginx_debug_msgs.msg import TwoDimensionalPlot, TwoDimensionalPlotDatapoint


class Main(AbstractNode):

    def initialization(self):
        self.rlock = threading.RLock()
        # self.debugplot_published = rospy.Publisher('planner/debug/plan', TwoDimensionalPlot, queue_size=2)
        self._path_publisher = rospy.Publisher('planner/debug/path', Path, queue_size=2)
        self._localization_publisher = rospy.Publisher('planner/debug/localization', PoseStamped, queue_size=2)
        self._target_publisher = rospy.Publisher('planner/debug/target_point', Marker, queue_size=2)
        self._clicked_point_publisher = rospy.Publisher('planner/debug/clicked_point', Marker, queue_size=2)
        # self.localization = Point()
        # self.path = list()

        rospy.Subscriber(rospy.get_param('/planner/topics/route/control'),
                         Route,
                         self.route_callback)
        rospy.Subscriber(rospy.get_param('/planner/topics/localization'),
                         Localization,
                         self.localization_callback)
        rospy.Subscriber(rospy.get_param('/planner/topics/task_to_point_planning'),
                         RouteTaskToPoint,
                         self.route_task_callback)
        rospy.Subscriber('/clicked_point',
                         PointStamped,
                         self.clicked_point_callback)

    def localization_callback(self, message):
        with self.rlock:
            pose_stamped = PoseStamped()
            pose_stamped.header = message.header
            pose_stamped.pose.position.x = message.pose.x
            pose_stamped.pose.position.y = message.pose.y
            quaternion = quaternion_from_euler(0, 0, message.yaw)
            pose_stamped.pose.orientation.x = quaternion[0]
            pose_stamped.pose.orientation.y = quaternion[1]
            pose_stamped.pose.orientation.z = quaternion[2]
            pose_stamped.pose.orientation.w = quaternion[3]
            self._localization_publisher.publish(pose_stamped)

    def route_callback(self, message):
        with self.rlock:
            path = Path()
            path.header = message.header
            path.header.frame_id = 'world'

            for p in message.route:
                pose_stamped = PoseStamped()
                pose_stamped.header = message.header
                pose_stamped.pose.position.x = p.x
                pose_stamped.pose.position.y = p.y

                path.poses.append(pose_stamped)

            self._path_publisher.publish(path)

    def route_task_callback(self, message):
        with self.rlock:
            # pose_stamped = PoseStamped()
            # pose_stamped.header = message.header
            # pose_stamped.header.frame_id = 'world'
            # pose_stamped.pose.position.x = message.target_pose.position.x
            # pose_stamped.pose.position.y = message.target_pose.position.y
            # self._target_publisher.publish(pose_stamped)
            marker = Marker()
            marker.header = message.header
            marker.header.frame_id = "world"
            # marker_.header.stamp = rospy.Time.now()
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.pose.position.x = message.target_pose.position.x
            marker.pose.position.y = message.target_pose.position.y
            marker.color.a = 1
            marker.color.r = 0.8
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.scale.x = 0.7
            marker.scale.y = 0.7
            marker.scale.z = 0.7

            self._target_publisher.publish(marker)

            # marker_.lifetime = rospy.Duration.from_sec(lifetime_)
            # marker_.scale.x = scale_[0]
            # marker_.scale.y = scale_[1]
            # marker_.scale.z = scale_[2]
            # marker_.color.a = 0.5
            # red_, green_, blue_ = color_
            # marker_.color.r = red_
            # marker_.color.g = green_
            # marker_.color.b = blue_

    def clicked_point_callback(self, message):
        with self.rlock:
            # pose_stamped = PoseStamped()
            # pose_stamped.header = message.header
            # pose_stamped.header.frame_id = 'world'
            # pose_stamped.pose.position.x = message.target_pose.position.x
            # pose_stamped.pose.position.y = message.target_pose.position.y
            # self._target_publisher.publish(pose_stamped)
            marker = Marker()
            marker.header = message.header
            marker.header.frame_id = "world"
            # marker_.header.stamp = rospy.Time.now()
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.pose.position.x = message.point.x
            marker.pose.position.y = message.point.y
            marker.color.a = 1
            marker.color.r = 0.80
            marker.color.g = 0.76
            marker.color.b = 0.00
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            # marker.color.r = red_
            # marker.color.g = green_
            # marker.color.b = blue_
            self._clicked_point_publisher.publish(marker)

            # marker_.lifetime = rospy.Duration.from_sec(lifetime_)
            # marker_.scale.x = scale_[0]
            # marker_.scale.y = scale_[1]
            # marker_.scale.z = scale_[2]
            # marker_.color.a = 0.5
            # red_, green_, blue_ = color_
            # marker_.color.r = red_
            # marker_.color.g = green_
            # marker_.color.b = blue_

    def work(self):
        pass
        # with self.rlock:
        #     point = TwoDimensionalPlotDatapoint()
        #     point.label = 'RobotPos'
        #     path = TwoDimensionalPlotDatapoint()
        #     path.label = 'RobotPath'

        #     point.data.append(self.localization)
        #     path.data = self.path

        #     message = TwoDimensionalPlot()
        #     message.title = "Path"
        #     message.points.append(point)
        #     message.lines.append(path)

        #     self.debugplot_published.publish(message)


if __name__ == '__main__':
    Main('webviz_adapter', 5).run()
