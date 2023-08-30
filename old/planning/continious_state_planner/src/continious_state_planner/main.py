import rospy

from .astar_statespace import AstarPathPlanner
from abstractnode import AbstractNode


class ContiniousStatePlanner(AbstractNode):
    def initialization(self):
        self.s = rospy.Service('path_request', CoveragePathSrv, self.handle_path_request)

    def handle_path_request(self, message):
        
        return {'response': response}