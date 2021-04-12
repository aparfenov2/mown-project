import rospy
import numpy as np

from abstractnode import AbstractNode
from coverage_path_planner import CoveragePathClient
from enjnx_msgs.msgs import GlobalTask, Localization, Route
from geometry_msgs.msgs import Point32, PoseStamped
from nav_msgs.msgs import Path


class LocalTrajectoryPlanner(AbstractNode):
    def initialization(self):
        pass

    def work(self):
        pass