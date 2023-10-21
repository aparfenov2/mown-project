import rospy
import random
import time
from coverage_path_planner import CoveragePathClient

from geometry_msgs.msg import Polygon, Point32
from enginx_msgs.msg import LocalTrajectoryStamped, Localization, Route, PointWithSpeed
from enginx_msgs.msg import Localization, PointWithSpeed, Route, RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner

if __name__ == '__main__':
    
    rospy.init_node('test_polygon_node')

    polygon = Polygon()
    source_point = Point32(x=-5.0, y=10.0)

    ext = [(-4, -4), (-4, 4), (4, 4), (4, -4), (0, -4)]

    for p in ext:
        point = Point32()
        point.x = p[0]
        point.y = p[1]
        polygon.points.append(point)

    msg = RouteTaskPolygon()
    msg.target_polygon = polygon

    polygon_taskpublisher = rospy.Publisher('/planner/route_task_polygon', RouteTaskPolygon, queue_size=10)
    polygon_taskpublisher_rviz = rospy.Publisher('/planner/test', Polygon, queue_size=10)
    polygon_taskpublisher.publish(msg)
    polygon_taskpublisher_rviz.publish(polygon)

    time.sleep(5.0)
    polygon_taskpublisher.publish(msg)
    polygon_taskpublisher_rviz.publish(polygon)


