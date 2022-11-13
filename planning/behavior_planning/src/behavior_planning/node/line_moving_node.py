from json import load
import time
import math

import dubins
import numpy as np
import rospy
from task_behavior_engine.tree import Node, NodeStatus
from .speed_generator import SpeedGenerator

from enginx_msgs.msg import Route, PointWithSpeed


class LineMovingNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(LineMovingNode, self).__init__(name=name,
                                             run_cb=self.run,
                                             *args, **kwargs)
        self._frame = frame
        self._last_stamp = 0.0
        self._speed_generator = None

    def run(self, nodedata):
        if (not self._frame.localization.has_localization()):
            rospy.loginfo_throttle(1, "Waiting for localization.")
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a localization")

        if (not self._frame.line_moving_task.has_message()):
            rospy.loginfo_throttle(1, "Waiting for LineMovingTask message.")
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a LineMovingTask message")

        lmt_stamp_time = self._frame.line_moving_task.stamp_sec
        ptt_stamp_time = self._frame.planning_task_type.stamp_sec

        rospy.loginfo_throttle(1, f"[LineMovingNode] {self._last_stamp=}, {lmt_stamp_time=}, {ptt_stamp_time=}")

        if (self._last_stamp < lmt_stamp_time
                and lmt_stamp_time >= ptt_stamp_time):
            target_speed = self._frame.line_moving_task.target_speed
            turning_radius = 3.0
            step_size = 0.1
            line_distance = self._frame.line_moving_task.distance

            pose = self._frame.localization.pose
            x_0 = pose
            x_f = (pose[0] + line_distance * np.cos(pose[2]),
                   pose[1] + line_distance * np.sin(pose[2]),
                   pose[2])

            path = dubins.shortest_path(x_0, x_f, turning_radius)
            configurations, _ = path.sample_many(step_size)

            route = Route()
            route.header.stamp = self._frame.localization.stamp
            for x, y, yaw in configurations:
                pws = PointWithSpeed()
                pws.x = x
                pws.y = y
                pws.d_time = 0.0
                route.route.append(pws)

            self._speed_generator = SpeedGenerator(target_speed, 5, 5)
            self._speed_generator.fill_speeds(route, True)

            self._frame.set_trajectory(route)
            self._last_stamp = self._frame.line_moving_task.stamp_sec            

        return NodeStatus(NodeStatus.SUCCESS)
