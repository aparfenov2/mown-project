import rospy

from abstractnode import AbstractNode
from geometry_msgs.msgs import Point32
from nav_msgs.msgs import Path
from coverage_path_planner.msgs import CoveragePathRequest, CoveragePathResponse
from coverage_path_planner.srv import CoveragePathSrv
from coverage_path_planner.area_polygon import AreaPolygon


class CoveragePathPlannerNode(AbstractNode):
    def initialization(self):
        s = rospy.Service('coverage_path_request', CoveragePathSrv, self.handle_coverage_path_request)

        self.ft = 0.2

    def handle_coverage_path_request(self, message):
        polygon = message.area_polygon
        source_point = message.source_point

        polygon_list = [(point.x, point.y) for point in polygon.points]

        polygon = AreaPolygon(polygon_list, source_point, ft=self.ft)
        route = polygon.get_area_coverage()

        response = self.__route2resp(route)
        return {'response': response}

    def __route2resp(self, route):
        resp = CoveragePathResponse()
        resp.header.stamp = rospy.get_rostime()
        for point in route.coords:
            ppoint = Point32()
            ppoint.x = point[0]
            ppoint.y = point[1]
            resp.path.append(ppoint)
        
        resp.response = resp

        return resp
