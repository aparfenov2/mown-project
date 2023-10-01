import rospy
import random
import time

from enginx_msgs.msg import LocalTrajectoryStamped, Localization, Route, PointWithSpeed


if __name__ == '__main__':
    
    rospy.init_node('test_node')
    r_pub = rospy.Publisher('/route_task', Route, queue_size=0)
    l_pub = rospy.Publisher('/localization', Localization, queue_size=0)

    pos = Localization()
    pos.pose.x, pos.pose.y = 5, 1

    route = Route()

    for i in range(-5, 15):
        p = PointWithSpeed()
        p.x = float(i)
        p.y = 0.01 * random.random()
        route.route.append(p)

    l_pub.publish(pos)
    time.sleep(0.5)
    r_pub.publish(route)
    time.sleep(0.5)
    l_pub.publish(pos)
    time.sleep(1.0)


