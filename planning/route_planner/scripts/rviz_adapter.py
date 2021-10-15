#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from enginx_msgs.msg import RouteTaskToPoint, Route

class Main:

    def __init__(self):
        rospy.init_node('nav_goal_msg_adapter')

        self.pub = rospy.Publisher(rospy.get_param('/planner/topics/task_to_point_planning'), RouteTaskToPoint, queue_size=2)
        self.path_pub = rospy.Publisher('planner/rviz/path', Path, queue_size=2)
        
        rospy.Subscriber(
            '/move_base_simple/goal', 
            PoseStamped,
            self.nav_goal_callback
        )     

        rospy.Subscriber(
            rospy.get_param('/planner/topics/route'), 
            Route,
            self.route_callback
        )

    def nav_goal_callback(self, msg):
        message = RouteTaskToPoint()
        message.header.stamp = rospy.get_rostime()
        pos = msg.pose.position
        message.target_pose.position.x = pos.x
        message.target_pose.position.y = pos.y
        self.pub.publish(message)


    def route_callback(self, message):
        path = Path()
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'world'

        for p in message.route:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = p.x
            pose.pose.position.y = p.y

            path.poses.append(pose)

        self.path_pub.publish(path)


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    Main().main()