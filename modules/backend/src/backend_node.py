import os
import cv2
import rospy
import sys
import roslib
import numpy as np
import yaml
import math
import json

import tf2_ros

import geometry_msgs.msg
import std_msgs.msg
import visualization_msgs.msg
from engix_msgs.msg import CoverageTask, DubinsPlanningTask
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32, PoseStamped

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
    return math.sqrt(
        math.pow(one.x - two.x, 2)
        + math.pow(one.y - two.y, 2)
        + math.pow(one.z - two.z, 2)
    )


def polygonPerimeter(polygon):
    perimeter = 0
    if len(polygon) > 1:
        # for (unsigned int i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
        j = len(polygon) - 1
        for i in range(len(polygon)):
            perimeter += pointsDistance(polygon[i], polygon[j])
            j = i
    return perimeter


def pointsNearby(one, two, proximity):
    return pointsDistance(one, two) <= proximity


def pointInPolygon(point, polygon):
    cross = 0
    #   for (unsigned int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++)
    j = len(polygon) - 1
    for i in range(len(polygon)):
        if ((polygon[i].y > point.y) != (polygon[j].y > point.y)) and (
            point.x
            < (polygon[j].x - polygon[i].x)
            * (point.y - polygon[i].y)
            / (polygon[j].y - polygon[i].y)
            + polygon[i].x
        ):
            cross += 1
        j = i
    return bool(cross % 2)


class BackendNode:
    def __init__(self):
        self.points = []
        self.pointsFrame = "unknown"
        self.polygon_is_built = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.map_frame = rospy.get_param('~map_frame', 'odom')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'base_link')

        self.point_viz_pub = rospy.Publisher(
            "exploration_polygon_marker", visualization_msgs.msg.Marker, queue_size=10
        )

        self.pub_move_to_point = rospy.Publisher(
            "/global_planner/dubins_planning_task",
            DubinsPlanningTask,
            queue_size=2,
        )
        self.pub = rospy.Publisher(
            "/global_planner/coverage_planning_task",
            CoverageTask,
            queue_size=2,
        )

        self.ui_updates_pub = rospy.Publisher(
            "/ui_updates",
            std_msgs.msg.String,
            queue_size=2,
        )

        self.pathgen_props = {
            "step_size": 0.1,
            "angle": 30,
            "auto_angle": True
        }

        self.ui_state = {
            "op_mode_state" : "manual",
            "progress": 10,
            "curr_speed": 0.5,
            "pathgen_props": self.pathgen_props
        }

        rospy.Timer(rospy.Duration(0.1), self.visTimerCb)
        rospy.Timer(rospy.Duration(0.1), self.uiUpdatesTimerCb)

        rospy.Subscriber("/clicked_point", PointStamped, self.pointCb)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.simple_goal_cb)
        rospy.Subscriber("/ui_cmd", std_msgs.msg.String, self.cmdUiCb)


    def cmdUiCb(self, cmd_msg: std_msgs.msg.String):
        jss = cmd_msg.data
        js = json.loads(jss)
        cmd = js["cmd"]

        if cmd == "switch_op_mode":
            self.ui_state["op_mode_state"] = "auto" if js["displayed_value"] == "manual" else "manual"

        if cmd == "add_poly":
            if not self.polygon_is_built:
                self.sendUserAlert("add_poly is not implemented. Finish building a poly")

        if cmd == "build_path":
            self.sendUserAlert("build_path is not implemented. Will be shown after Start.")

        if cmd == "start":
            rospy.loginfo("emit execute msg")
            if not self.polygon_is_built:
                rospy.logerr("cannot execute: build polygon first")
                self.sendUserAlert("cannot execute: build polygon first")
                return
            assert len(self.points) > 2, str(len(self.points))
            message = CoverageTask()
            message.header.stamp = rospy.get_rostime()
            message.auto_angle = True
            message.approximate = True
            for p in self.points:
                new_point = Point32()
                new_point.x = p.x
                new_point.y = p.y
                message.target_polygon.points.append(new_point)
            self.pub.publish(message)

        if cmd == "stop":
            self.sendUserAlert("stop is not implemented")

        if cmd == "reset":
            rospy.loginfo("reset called")
            self.polygon_is_built = False
            self.points = []

        if cmd == "set_desired_speed":
            self.ui_state["curr_speed"] = float(js["desired_speed"])

        if cmd == "set_pathgen_props":
            self.pathgen_props["step_size"] = float(js["step_size"])
            self.pathgen_props["angle"] = float(js["angle"])
            self.pathgen_props["auto_angle"] = bool(js["auto_angle"])

    def sendUserAlert(self, msg):
        js = {
            "error": msg
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(js)
        self.ui_updates_pub.publish(msg)

    def uiUpdatesTimerCb(self, arg0):
        js = {
            "state": self.ui_state
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(js)
        self.ui_updates_pub.publish(msg)

    def get_current_pos(self):
        trans = self.tfBuffer.lookup_transform(self.map_frame, self.base_link_frame, rospy.Time())
        return trans.transform.translation.x, trans.transform.translation.y

    def simple_goal_cb(self, msg: PoseStamped):
        message = DubinsPlanningTask()
        message.header.stamp = rospy.get_rostime()
        # x, y = self.get_current_pos()
        trg_x, trg_y = msg.pose.position.x, msg.pose.position.y
        message.target_pose.x = trg_x
        message.target_pose.y = trg_y
        message.target_speed  = self.ui_state["curr_speed"]
        message.turning_radius  = 1.0
        message.step_size  = 0.5
        self.pub_move_to_point.publish(message)
        rospy.loginfo("Published DubinsPlanningTask msg, dist=%f, spd=%f", message.distance, message.target_speed)


    def pointCb(self, point_msg: PointStamped):
        point = point_msg.point
        if self.points:
            average_distance = polygonPerimeter(self.points) / len(self.points)
        else:
            average_distance = 10

        if self.polygon_is_built:
            rospy.logwarn("Polygon is built. Start creating new")
            self.polygon_is_built = False
            self.points = []

        if not self.points:
            self.pointsFrame = point_msg.header.frame_id
            self.points += [point]

        elif self.pointsFrame != point_msg.header.frame_id:
            rospy.logerr("Frame mismatch, restarting polygon selection")
            self.points = []

        elif len(self.points) > 1 and pointsNearby(
            self.points[0], point, average_distance * 0.1
        ):
            if len(self.points) < 3:
                rospy.logerr("Not a valid polygon, restarting")
                self.points = []
            else:
                rospy.loginfo("Polygon is built. Please press Go button")
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
            points.action = line_strip.action = visualization_msgs.msg.Marker.ADD
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0

            points.scale.x = points.scale.y = 0.1
            line_strip.scale.x = 0.05

            for point in self.points:
                line_strip.points += [point]
                points.points += [point]

            if self.polygon_is_built:
                line_strip.points += [line_strip.points[0]]
                points.color.a = (
                    points.color.r
                ) = line_strip.color.r = line_strip.color.a = 1.0
            else:
                points.color.a = (
                    points.color.b
                ) = line_strip.color.b = line_strip.color.a = 1.0
        else:
            points.action = line_strip.action = visualization_msgs.msg.Marker.DELETE

        self.point_viz_pub.publish(points)
        self.point_viz_pub.publish(line_strip)

    def visTimerCb(self, arg0):
        self.publishCurrentPoints()

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("backend_node", anonymous=True)
    BackendNode().main()
