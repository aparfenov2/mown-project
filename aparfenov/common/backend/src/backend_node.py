import os
import cv2
import rospy
import sys
import roslib
import numpy as np
import yaml
import math

import geometry_msgs.msg
import std_msgs.msg
import visualization_msgs.msg
from enginx_msgs.msg import RouteTaskToPoint, RouteTaskPolygon
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32

"""
interface Backend
services:
    - list_saved_maps
        - returns: list of map ids, names

    - load_map(map_id)
    passthrough: map topic, RGBGrid

    - set_coverage_polygon(points)
    passthrough: path topic, Path

    passthrough: camera stream

# joystick control
    - start_stop_motion()
    - inc_dec_speed()
    - turn_lr(lr)
"""

"""
нода реализует несколько сервисов, запрашиваемых через rosbridge с фронтенда,
такие как получить список карт, загрузить карту, загрузить полигон

case 2: track robot path and build convex polygon
"""

def pointsDistance(one, two):
    return math.sqrt(math.pow(one.x - two.x, 2) + math.pow(one.y - two.y, 2) + math.pow(one.z - two.z, 2));

def polygonPerimeter(polygon):
    perimeter = 0;
    if len(polygon) > 1:
        # for (unsigned int i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
        j = len(polygon) - 1
        for i in range(len(polygon)):
            perimeter += pointsDistance(polygon[i], polygon[j]);
            j = i
    return perimeter;

def pointsNearby(one, two, proximity):
    return pointsDistance(one, two) <= proximity;

def pointInPolygon(point, polygon):
    cross = 0;
#   for (unsigned int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++)
    j = len(polygon) - 1
    for i in range(len(polygon)):
        if  ((polygon[i].y > point.y) != (polygon[j].y > point.y)) and \
            (point.x < (polygon[j].x-polygon[i].x) * (point.y-polygon[i].y) /
            (polygon[j].y-polygon[i].y) + polygon[i].x) :
            cross+=1
        j = i
    return bool(cross % 2);


class BackendNode:
    def __init__(self):
        self.points = []
        self.pointsFrame = "unknown"
        self.polygon_is_built = False;

        rospy.init_node('backend_node', anonymous=True)

        # class _A: pass
        # self.args = _A()
        # self.args.rate = rospy.get_param("~rate")
        # self.args.cam_info_yaml = rospy.get_param("~cam_info_yaml")
        # self.FRAMES_FOLDER = rospy.get_param("~frames_folder")
        # self.DEPTH_FOLDER = rospy.get_param("~depth_folder")
        rospy.Subscriber("/clicked_point", geometry_msgs.msg.PointStamped, self.pointCb)
        rospy.Subscriber("/ui_cmd", std_msgs.msg.String, self.cmdUiCb)
        self.point_viz_pub = rospy.Publisher("exploration_polygon_marker", visualization_msgs.msg.Marker, queue_size=10)
        # self.pub = rospy.Publisher(rospy.get_param('/planner/topics/task_polygon_planning'), RouteTaskPolygon, queue_size=2)
        rospy.Timer(rospy.Duration(0.1), self.visTimerCb)

    def cmdUiCb(self, cmd_msg: std_msgs.msg.String):
        cmd = cmd_msg.data
        if cmd == 'go':
            rospy.loginfo("emit execute msg");
            if not self.polygon_is_built:
                rospy.logerr("cannot execute: build polygon first");
                return
            assert len(self.points) > 2, str(len(self.points))
            message = RouteTaskPolygon()
            message.header.stamp = rospy.get_rostime()
            for p in self.points:
                new_point = Point32()
                new_point.x = p.x
                new_point.y = p.y
                message.target_polygon.points.append(new_point)
            # self.pub.publish(message)

    def pointCb(self, point_msg: geometry_msgs.msg.PointStamped):
        point = point_msg.point
        # self.points.append((point.x, point.y))
        if self.points:
            average_distance = polygonPerimeter(self.points) / len(self.points)
        else:
            average_distance = 10

        if self.polygon_is_built:
            rospy.logwarn("Polygon is built. Start creating new");
            self.polygon_is_built = False
            self.points = []

        if not self.points:
            self.pointsFrame = point_msg.header.frame_id
            self.points += [point]

        elif self.pointsFrame != point_msg.header.frame_id:
            rospy.logerr("Frame mismatch, restarting polygon selection");
            self.points = []

        elif len(self.points) > 1 and pointsNearby(self.points[0], point, average_distance*0.1):
            if len(self.points) < 3:
                rospy.logerr("Not a valid polygon, restarting");
                self.points = []
            else:
                rospy.loginfo("Polygon is built. Please press Go button");
                self.polygon_is_built = True

        else:
            self.points += [point]

        self.publishCurrentPoints()

    def publishCurrentPoints(self):
        _time = rospy.Time.now()
        points = visualization_msgs.msg.Marker()
        points.header.stamp = _time
        points.header.frame_id = self.pointsFrame
        points.id = 0
        points.lifetime = rospy.Duration()
        points.ns = "explore_points"
        points.type = visualization_msgs.msg.Marker.SPHERE_LIST

        line_strip = visualization_msgs.msg.Marker()
        line_strip.header.stamp = _time
        line_strip.header.frame_id = self.pointsFrame
        line_strip.id = 1
        line_strip.lifetime = rospy.Duration()
        line_strip.ns = "explore_points"
        line_strip.type = visualization_msgs.msg.Marker.LINE_STRIP

        if self.points:
            points.action = line_strip.action = visualization_msgs.msg.Marker.ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            points.scale.x = points.scale.y = 0.1;
            line_strip.scale.x = 0.05;

            for point in self.points:
                line_strip.points += [point]
                points.points += [point]

            if self.polygon_is_built:
                line_strip.points += [line_strip.points[0]]
                points.color.a = points.color.r = line_strip.color.r = line_strip.color.a = 1.0;
            else:
                points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;
        else:
            points.action = line_strip.action = visualization_msgs.msg.Marker.DELETE;

        self.point_viz_pub.publish(points);
        self.point_viz_pub.publish(line_strip);


    def visTimerCb(self, arg0):
        self.publishCurrentPoints()

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    BackendNode().main()
