import numpy as np
import rospy

from abstractnode import AbstractNode
from geometry_msgs.msg import Point32
from nav_msgs.msg import Path
from coverage_path_planner.msg import CoveragePathRequest, CoveragePathResponse
from coverage_path_planner.srv import CoveragePathSrv
from coverage_path_planner.area_polygon import AreaPolygon


class CoveragePathPlannerNode(AbstractNode):
    def initialization(self):
        self.s = rospy.Service('coverage_path_request', CoveragePathSrv, self.handle_coverage_path_request)

        self.ft = 0.5

    def handle_coverage_path_request(self, message):
        message = message.request
        print(message.polygon)

        polygon = message.polygon
        source_point = np.array([message.source_point.x, message.source_point.y])

        polygon_list = [(point.x, point.y) for point in polygon.points]

        extended_polygon = []

        for i in range(1, len(polygon_list)):
            extended_polygon.append(polygon_list[i-1])
            point_count = np.linalg.norm(np.array(polygon_list[i]) - np.array(polygon_list[i-1]))
            point_count = max(1, int(point_count))
            extended_segment = np.linspace(polygon_list[i-1], polygon_list[i], point_count).tolist()
            extended_polygon += extended_segment

        extended_polygon.append(polygon_list[-1])

        polygon = AreaPolygon(extended_polygon, source_point, ft=self.ft)
        # rospy.logarn(f"Polygon: {polygon}")
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

        # resp.response = resp

        return resp
