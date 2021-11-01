#!/usr/bin/env python3

import numpy as np
import rospy
import ros_numpy
from tf import TransformListener

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from enginx_msgs.msg import RouteTaskToPoint, Route


def cloud_msg_to_numpy(cloud_msg):
    # Convert Ros pointcloud2 msg to numpy array
    pc = ros_numpy.numpify(cloud_msg)
    points=np.zeros((pc.shape[0],4))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    cloud = np.array(points, dtype=np.float32)

    return cloud


def np2ros_pub_2(points, frame_id, timestamp):
    npoints = points.shape[0] # Num of points in PointCloud
    points_arr = np.zeros((npoints,), dtype=[
                                        ('x', np.float32),
                                        ('y', np.float32),
                                        ('z', np.float32),
                                        ('intensity', np.float32)])

    points = np.transpose(points)
    points_arr['x'] = points[0]
    points_arr['y'] = points[1]
    points_arr['z'] = points[2]
    points_arr['intensity'] = points[3]
    
    cloud_msg = ros_numpy.msgify(PointCloud2, points_arr,stamp=timestamp, frame_id=frame_id)
    return cloud_msg



class Main:

    def __init__(self):
        rospy.init_node('nav_goal_msg_adapter')

        self.pub = rospy.Publisher(rospy.get_param('/planner/topics/task_to_point_planning'), RouteTaskToPoint, queue_size=2)
        self.path_pub = rospy.Publisher('planner/rviz/path', Path, queue_size=2)
        self.pcl_pub = rospy.Publisher("/velodyne_points/new", PointCloud2, queue_size=10)
        
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

        rospy.Subscriber(
            '/velodyne_points', 
            PointCloud2,
            self.poitcloude_callback
        )

    def nav_goal_callback(self, msg):
        message = RouteTaskToPoint()
        message.header.stamp = rospy.get_rostime()
        pos = msg.pose.position
        message.target_pose.position.x = pos.x
        message.target_pose.position.y = pos.y
        self.pub.publish(message)

    def poitcloude_callback(self, message):
        cloud = cloud_msg_to_numpy(message)
        cloud = cloud[np.where(cloud[:, 2] > 1.25 - 1.733)]

        cloude_msg = np2ros_pub_2(cloud, message.header.frame_id, message.header.stamp)
        self.pcl_pub.publish(cloude_msg)

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