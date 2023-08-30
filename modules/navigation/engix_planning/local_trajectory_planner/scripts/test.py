import rospy
import random
import time

from engix_msgs.msg import RouteTaskPolygon
from geometry_msgs.msg import Polygon, Point32


if __name__ == '__main__':
    
    rospy.init_node('test_node_n')
    r_pub = rospy.Publisher('/planner/route_task_polygon', RouteTaskPolygon, queue_size=0)
    # l_pub = rospy.Publisher('/localization', Localization, queue_size=0)

    route = RouteTaskPolygon()
    route.target_polygon.points.append(Point32(
        x=-2.0,
        y=2.0
    ))
    route.target_polygon.points.append(Point32(
        x=2.0,
        y=2.0
    ))
    route.target_polygon.points.append(Point32(
        x=2.0,
        y=-2.0
    ))
    route.target_polygon.points.append(Point32(
        x=-2.0,
        y=-2.0
    ))

    time.sleep(0.5)
    r_pub.publish(route)
    time.sleep(0.5)
    r_pub.publish(route)
    time.sleep(0.5)
