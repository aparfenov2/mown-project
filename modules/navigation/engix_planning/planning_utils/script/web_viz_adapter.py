#!/usr/bin/env python3

import numpy as np
import rospy
import ros_numpy
from tf import TransformListener
import threading

from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from abstractnode import AbstractNode
from engix_msgs.msg import RouteTaskToPoint, Route, Localization
from enginx_debug_msgs.msg import TwoDimensionalPlot, TwoDimensionalPlotDatapoint


class Main(AbstractNode):

    def initialization(self):
        self.rlock = threading.RLock()
        self.debugplot_published = rospy.Publisher('planner/debug/plan', TwoDimensionalPlot, queue_size=2)

        self.localization = Point()
        self.path = list()

        rospy.Subscriber(rospy.get_param('/planner/topics/route/control'), 
                         Route, 
                         self.route_callback)
        rospy.Subscriber(rospy.get_param('/planner/topics/localization'), 
                         Localization, 
                         self.odometry_callback)

    def odometry_callback(self, message):
        with self.rlock:
            self.localization = Point()
            self.localization.x = message.pose.x
            self.localization.y = message.pose.y

    def route_callback(self, message):
        with self.rlock:
            self.path = list()

            for p in message.route:
                pose = Point()
                pose.x = p.x
                pose.y = p.y

                self.path.append(pose)

    def work(self):
        with self.rlock:
            point = TwoDimensionalPlotDatapoint()
            point.label = 'RobotPos'
            path = TwoDimensionalPlotDatapoint()
            path.label = 'RobotPath'

            point.data.append(self.localization)
            path.data = self.path

            message = TwoDimensionalPlot()
            message.title = "Path"
            message.points.append(point)
            message.lines.append(path)

            self.debugplot_published.publish(message)

if __name__ == '__main__':
    Main('web_viz_adapter', 5).run()