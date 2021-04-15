import rospy
from coverage_path_planner.msgs import CoveragePathRequest, CoveragePathResponse
from coverage_path_planner.srv import CoveragePathSrv


class CoveragePathClient(object):
    def __init__(self):
        rospy.wait_for_service('coverage_path_request')
        self.server_request = rospy.ServiceProxy('coverage_path_request', CoveragePathSrv)

    def get_path(self, polygon, source_point):
        try:
            request = CoveragePathRequest(polygon=polygon, source_point=source_point)
            
            resp1 = self.server_request(request)
            return resp1.response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)