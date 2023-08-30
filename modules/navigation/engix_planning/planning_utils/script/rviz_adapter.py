#!/usr/bin/env python3

import numpy as np
import rospy
import ros_numpy
from tf import TransformListener

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from engix_msgs.msg import RouteTaskToPoint, Route


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


class GroundFiltering(object):

    def filter_point_cloud(self, points, threshold, max_iter, max_inner_rate):
        N = points.shape[0]
        max_inner_num = np.floor(N*max_inner_rate)  # If the number of internal points reaches max_inner_num, stop in advance
        inner_max_num = 0
        plane = np.zeros((4, 1), dtype=float)
        for i in range(max_iter):
            random_idx = np.random.choice(np.arange(N), size=3, replace=False)  # Do not repeat three integers
            sample_points = points[random_idx, :]
            sample_plane = self.get_plane_by_lsq(sample_points)
            inner_points = self.get_inner(points, sample_plane, threshold)
            inner_num = self.get_inner_num(inner_points)
            # If you meet the early termination conditions, then terminate the program in advance
            # early stop condition
            if inner_num > max_inner_num:
                # After satisfying the condition, the least squares of the internal point is performed again to improve the accuracy.
                plane = self.get_plane_by_lsq(inner_points)
                break
            # If the number of internal points is larger than the maximum, then update the Plane parameter, and update the internal point maximum number inner_max_num Set Inner_Num
            if inner_num > inner_max_num:
                plane = self.get_plane_by_lsq(inner_points)
                inner_max_num = inner_num
            # Continue iteration, Continue statement looks beautiful
            continue
        #           
        idx = self.get_without_plane_idx(points, plane, threshold)
        return idx

    def get_inner_num(self, inner_points):
        return inner_points.shape[0]

    # Plane: Plane parameter [A, B, C, D], meet A ^ 2 + B ^ 2 + C ^ 2 = 1, array form
    def get_plane_by_lsq(self, data):
        # hstack: Array in the horizontal stacking sequence, here is equivalent to adding a column after DATA 1
        A = np.hstack((data, np.ones((data.shape[0], 1), dtype=np.float64)))
        AtA = np.matmul(A.T, A)
        # # A                                                                
        #       | |                                                                      
        U, S, V_H = np.linalg.svd(AtA)
        # The minimum feature value corresponds to the characteristic vector of the output, normalized, normalized, so that [a, b, c] is unit vector
        plane = U[:, -1] / np.linalg.norm(U[0:3, -1])
        return plane
    
    def get_inner(self, points, plane, threshold):
        idx = self.get_inner_idx(points, plane, threshold)
        #  : Here Return Points [IDX,:] can also, Python seems to
        return points[idx]

    def get_without_plane(self, points, plane, threshold):
        idx = self.get_without_plane_idx(points, plane, threshold)
        #  : Here Return Points [IDX,:] can also, Python seems to
        return points[idx]

    def get_inner_idx(self, points, plane, threshold):
        N = points.shape[0]
        data = np.hstack((points, np.ones((N, 1), dtype=np.float32)))
        distance = abs(np.matmul(data, plane)) / np.linalg.norm(plane[0:3])
        return distance < threshold

    def get_without_plane_idx(self, points, plane, threshold):
        N = points.shape[0]
        data = np.hstack((points, np.ones((N, 1), dtype=np.float32)))
        distance = abs(np.matmul(data, plane)) / np.linalg.norm(plane[0:3])
        return distance > threshold



class Main:

    def __init__(self):
        rospy.init_node('nav_goal_msg_adapter')

        self.pub = rospy.Publisher(rospy.get_param('/planner/topics/task_to_point_planning'), RouteTaskToPoint, queue_size=2)
        self.path_pub = rospy.Publisher('planner/rviz/path', Path, queue_size=2)
        # self.pcl_pub = rospy.Publisher("/velodyne_points/new", PointCloud2, queue_size=10)
        
        rospy.Subscriber(
            '/move_base_simple/goal', 
            PoseStamped,
            self.nav_goal_callback
        )     

        rospy.Subscriber(
            rospy.get_param('/planner/topics/route/control'), 
            Route,
            self.route_callback
        ) 

        # rospy.Subscriber(
        #     '/velodyne_points', 
        #     PointCloud2,
        #     self.pointcloude_callback
        # )

    def nav_goal_callback(self, msg):
        message = RouteTaskToPoint()
        message.header.stamp = rospy.get_rostime()
        pos = msg.pose.position
        message.target_pose.position.x = pos.x
        message.target_pose.position.y = pos.y
        self.pub.publish(message)

    def pointcloude_callback(self, message):
        cloud = cloud_msg_to_numpy(message)
        plane_filter = GroundFiltering()
        idx = plane_filter.filter_point_cloud(cloud[:, :3], 0.5, 40, 100)
        cloud = cloud[idx]
        # cloud = cloud[np.where(cloud[:, 2] > 1.25 - 1.733)]

        print('Cloud shape', cloud.shape)

        cloud_msg = np2ros_pub_2(cloud, message.header.frame_id, message.header.stamp)
        self.pcl_pub.publish(cloud_msg)

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