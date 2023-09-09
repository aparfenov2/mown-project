#!/usr/bin/env python3

import threading
import rospy
import time

from abstractnode import AbstractNode
from global_planner.common.frame import Frame
from global_planner.planners.straight_line_planner import StraightLinePlanner
from global_planner.planners.dubins_path_planner import DubinsPathPlanner
from global_planner.planners.coverage_path_planner import CoveragePathPlanner

from nav_msgs.msg import Path
from engix_msgs.msg import (
    Localization,
    CoverageTask,
    LineMovingTask,
    DubinsPlanningTask,
)
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point32


if __name__ == "__main__":
    rospy.init_node("publish_coverage_task_mock")
    time.sleep(0.2)


    path_publisher = rospy.Publisher(
        "/global_planner/coverage_planning_task", CoverageTask, queue_size=10, latch=True
    )

    message = CoverageTask()
    message.header.stamp = rospy.get_rostime()

    message.angle = 0.0
    message.approximate = True
    message.auto_angle = True

    message.target_polygon.points.append(Point32(5.0, 3.0, 0.0))
    message.target_polygon.points.append(Point32(-5.0, 3.0, 0.0))
    message.target_polygon.points.append(Point32(-5.0, 6.0, 0.0))
    message.target_polygon.points.append(Point32(5.0, 6.0, 0.0))
    time.sleep(0.2)

    path_publisher.publish(message)
    time.sleep(0.2)

    rospy.spin()
