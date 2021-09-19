#!/usr/bin/env python3

import argparse
import time

import rospy

from enginx_msgs.msg import RouteTaskPolygon, RouteTaskToPoint
from geometry_msgs.msg import Point32


class PlannerTaskGenerateNode(object):
    def __init__(self):
        pass

def coords(s):
    print(s)
    try:
        x, y = map(float, s.split(','))
        return x, y
    except:
        raise argparse.ArgumentTypeError("Coordinates must be x,y")

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--point', help="Set target coordinate, example: --point 1.0,2.0", type=coords, nargs="*", default=None)
    parser.add_argument('--polygon', help="Set target coordinate, example: --polygon 1.0,2.0 3.0,5.0 7.0,6.0", type=coords, nargs="*", default=None)
    # parser.add_argument("--polygon", action="store", default=None)
    # parser.add_argument("--point", nargs="2", default=None)
    # args = parser.parse_args() 
    # args = parser.parse_args(["--polygon", "1,2", "4,5", "6,7"])
    args = parser.parse_args()

    if args.point is not None and args.polygon is not None:
        pass
        exit(1)

    if args.point is None and args.polygon is None:
        pass
        exit(1)


    if args.point is not None:
        if len(args.point) != 1:
            print('')
            exit(1)

        args.point = args.point[0]

    if args.polygon is not None and len(args.polygon) < 3:
        print()
        exit(1)


    return args


def main():
    rospy.init_node("test_plan_publisher")
    args = parse_arguments()
    r_pub = rospy.Publisher('/planner/route_task_polygon', RouteTaskPolygon, queue_size=0)
    rp_pub = rospy.Publisher('/planner/route_task_to_point', RouteTaskToPoint, queue_size=0)

    if args.point is not None:
        message = RouteTaskToPoint()
        message.header.stamp = rospy.get_rostime()
        point = args.point
        message.target_pose.position.x = point[0]
        message.target_pose.position.y = point[1]
        time.sleep(0.5)
        rp_pub.publish(message)
        time.sleep(0.5)
        rp_pub.publish(message)
        print(message)
        time.sleep(0.5)
    else:
        route = RouteTaskPolygon()
        route.header.stamp = rospy.get_rostime()
        for point in args.polygon:
            route.target_polygon.points.append(Point32(
                x=point[0],
                y=point[1]
            ))
        time.sleep(0.5)
        r_pub.publish(route)
        time.sleep(0.5)
        r_pub.publish(route)
        time.sleep(0.5)


if __name__ == '__main__':
    main()