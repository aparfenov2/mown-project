import rospy
import random
import time
from coverage_path_planner import CoveragePathClient

from geometry_msgs.msg import Polygon, Point32
from enginx_msgs.msg import LocalTrajectoryStamped, Localization, Route, PointWithSpeed


if __name__ == '__main__':
    
    rospy.init_node('test_node')
    client = CoveragePathClient()

    polygon = Polygon()
    source_point = Point32(x=-5.0, y=10.0)

    ext = [(0, 0), (4, 4), (5, 6), (0, 8), (-4, 4)]

    for p in ext:
        point = Point32()
        point.x = p[0]
        point.y = p[1]
        polygon.points.append(point)
    
    

    path = client.get_path(polygon, source_point)
    print(path)
    time.sleep(0.5)


