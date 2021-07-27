#!/usr/bin/env python3
from math import cos
from threading import RLock

import rospy
import actionlib
import numpy as np
from scipy import optimize
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from abstractnode import AbstractNode

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TwistStamped, Twist, Vector3
from move_base_mod.msg import TrajectoryControllerAction, TrajectoryControllerGoal
from nav_msgs.msg import Odometry

from enginx_msgs.msg import LocalTrajectoryStamped, Localization, Route


def convert_orientation(theta):
    return angles_difference(0.0, theta)


def angles_difference(a1, a2):
    a = a2 - a1

    if a > np.pi:
        a -= 2*np.pi
    elif a < -np.pi:
        a += 2*np.pi
    
    return a


def compute_theta(p1, p2):
    return np.math.atan2(p2[1] - p1[1], p2[0] - p1[0])


class ControllerNode(AbstractNode):
    
    def initialization(self):
        self.__lock = RLock()
        self.__last_loc = None
        self.__last_trajectory = None

        self.mpc = Controller2(10, 0.1, 0.1, 1.0, 1.0, 1.0, 1.0)
        self.speed_control = SpeedController(0.2)

        self.control_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.last_linear_speed = 0
        self.last_angular_speed = 0

        self.speed = 0.0
        self.last_idx = -1

        # rospy.Subscriber('/local_trajectory_plan', LocalTrajectoryStamped, self.__route_callback)
        rospy.Subscriber('/route', Route, self.__route_task_callback)
        # rospy.Subscriber('/laser_odom_to_init', Odometry, self.__odometry_callback)
        rospy.Subscriber('/localization', Localization, self.__odometry_callback)

    def local_trajectory_callback(self, message: LocalTrajectoryStamped):
        with self.__lock:
            poses = list()
            for pose in message.route:
                poses.append(self.__Posed2DtoPoseStamped(pose))

            self.__last_trajectory = message

        # self.send_msg(poses)

    def __Posed2DtoPoseStamped(self, pose2d):
        pose = Pose()
        pose.position.x = pose2d.x
        pose.position.y = pose2d.y
        pose_or = quaternion_from_euler(0, 0, pose2d.theta)
        pose.orientation.x = pose_or[0]
        pose.orientation.y = pose_or[1]
        pose.orientation.z = pose_or[2]
        pose.orientation.w = pose_or[3]

        posestamped = PoseStamped()
        posestamped.pose = pose
        posestamped.header.stamp = rospy.get_rostime()

        return posestamped

    def __route_task_callback(self, message):
        with self.__lock:
            print('Got message')
            point_list = list()

            for point in message.route:
                point_list.append([point.x, point.y, point.speed])
            
            self.__last_trajectory = point_list

            if len(self.__last_trajectory) == 0:
                self.__last_trajectory = None

            self.last_idx = -1

    def __odometry_callback(self, message: Odometry):
        with self.__lock:
            self.__last_loc = message
            self.speed = message.speed

    def send_msg(self, msg : list):
        goal = TrajectoryControllerGoal()
        goal.poses = msg
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()

    def work(self):
        with self.__lock:
            if self.__last_loc is None or self.__last_trajectory is None:
                self.send_idle_control()
                return 

            current_pos = (self.__last_loc.pose.x, self.__last_loc.pose.y, self.__last_loc.yaw)

            if self.__close_to_end(current_pos):
                self.__last_trajectory = None
                self.send_idle_control()
                self.last_idx = -1
                return

            target_point = self.__get_target_point(current_pos)

            print("Cur: {0}, target: {1}".format(
                current_pos, target_point
            ))
            linear_velocity = self.__linear_velocity_control(current_pos, target_point)
            angular_velocity = self.__angular_velocity_control(current_pos, target_point)

            linear_velocity, angular_velocity = self.mpc_control(
                [current_pos[0], current_pos[1]],
                [target_point[0], target_point[1]],
                current_pos[2],
                target_point[2]
            )

            self.send_control(linear_velocity, angular_velocity)

    def __close_to_end(self, pos):
        p1 = np.array(pos[:2])
        p2 = np.array(self.__last_trajectory[-1][:2])
        return np.linalg.norm(p1 - p2) < 0.49

    def mpc_control(self, initial_pos, target_point, yaw, target_speed):
        path = np.array([p[:2] for p in self.__last_trajectory])
        res = self.mpc.calc_control(initial_pos, path, yaw, self.speed)
        speed = self.speed_control.control_speed(initial_pos, yaw, self.speed, target_point, target_speed)

        # print('Optim control: {}'.format(res))

        theta = compute_theta(initial_pos, target_point)
        # print("Target theta: {0}, current yaw: {1}, diff:{2}".format(
        #     theta, 
        #     convert_orientation(yaw),
        #     angles_difference(convert_orientation(yaw), theta)))

        if res is None:
            dw = angles_difference(convert_orientation(yaw), theta)
            v, w = 0, dw
            rospy.logerr('No optimal control')
        else:
            v, w = res

        v = speed

        print("COntrol: {0}, {1}".format(v, w))

        linear_vel = Vector3(x=v, y=0.0, z=0.0)
        angular_vel = Vector3(x=0.0, y=0.0, z=w)
        self.last_linear_speed = v
        self.last_angular_speed = w
        return linear_vel, angular_vel         

    def send_control(self, linear_velocity, angular_velocity):
        message = Twist()
        # message.header.stamp = rospy.get_rostime()
        message.linear = linear_velocity
        message.angular = angular_velocity
        self.control_publisher.publish(message)

    def send_idle_control(self):
        message = Twist()
        self.control_publisher.publish(message)

    def __get_target_point(self, cur_pose):
        # poses = self.__last_trajectory.route
        path = np.array([p[:2] for p in self.__last_trajectory])
        cur_pose = np.array(cur_pose[:2])


        idx = (np.linalg.norm(path - cur_pose, axis=1)).argmin()
        idx = max(idx, self.last_idx)
        self.last_idx = idx
        path = np.array(self.__last_trajectory[idx:])

        if len(path) == 1:
            return path[0]

        distances = np.linalg.norm(path - path[0], axis=1)
        distances = np.abs(distances - 0.3)
        target_idx = distances.argmin()
        return path[target_idx]

    def __linear_velocity_control(self, current_pos, target_point):

        if np.linalg.norm(np.array(current_pos[:2]) - np.array(target_point[:2])) < 0.5:
            return Vector3(x=0.0, y=0.0, z=0.0)

        # if abs(current_pos[2] - target_point[2]) > 0.

        return Vector3(x=1.0, y=0.0, z=0.0)

    def __angular_velocity_control(self, current_pos, target_point):
        if abs(current_pos[2] - target_point[2]) < 0.1:
            return Vector3(x=0.0, y=0.0, z=0.0)

        rotate = 1.0 if current_pos[2] - target_point[2] > 0.0 else -1.0
        return Vector3(x=0.0, y=0.0, z=rotate)


class Controller:
    def __init__(self, horizon, dt, distance_threshold, dist_coef, lin_speed_coef, ang_speed_coef, speed_coef, dyaw_coef=1.0):
        self.horizon = horizon
        self.dt = dt
        self.distance_threshold = distance_threshold
        self.dist_coef = dist_coef
        self.lin_speed_coef = lin_speed_coef
        self.ang_speed_coef = ang_speed_coef
        self.speed_coef = speed_coef
        self.dyaw_coef = dyaw_coef

        self.x0 = [0.0] * (2 * self.horizon)
        self.bounds = list()
        for _ in range(self.horizon):
            self.bounds += [[-0.0, 0.50]]
            self.bounds += [[-0.50, 0.50]]

    def step(self, pos, yaw, linear_speed, angular_speed, dt):
        # a = (last_speed - linear_speed)
        x, y = pos
        x = x + linear_speed * np.math.cos(yaw) * dt
        y = y + linear_speed * np.math.sin(yaw) * dt
        yaw = yaw + angular_speed * dt
        return [x, y], yaw

    def cost_function(self, u, initial_pos, target_point, yaw, linear_speed, angular_speed, target_speed):
        cur_pos = [initial_pos[0], initial_pos[1]]
        cur_yaw = yaw
        # cur_ls, cur_as = u
        # linear_speed += dls
        # angular_speed += das
        fv = 0
        lv, lw = linear_speed, 0.0
        du = 0.0
        dyaw = 0.0

        cost = 0.0

        for i in range(self.horizon):
            v, w = u[i*2], u[(i) * 2 + 1]
            cur_pos, cur_yaw = self.step(cur_pos, cur_yaw, v, w, self.dt)
            fv = v
            if lv is not None:
                du = self.lin_speed_coef * ((lv - v) ** 2) + self.ang_speed_coef * ((lw - w) ** 2)
            # dyaw += 
            lv = v
            lw = w

            cost += (cur_pos[0] - target_point[0]) ** 2 + (cur_pos[1] - target_point[1]) ** 2
            # cost += du
            cost += (v - target_speed) ** 2
            
            if self.distance_btw(cur_pos, target_point) < self.distance_threshold:
                break

        # print("Control: {0}, cost: {1}, end pos: {2}".format(
        #     u,
        #     self.dist_coef * self.distance_btw(cur_pos, target_point) + \
        #         du + \
        #         self.speed_coef * np.math.fabs(fv - target_speed),
        #     cur_pos
        # ))
        # print("Start: {0}, end: {1}, target:{2}, dist to: {3}, cost: {4}".format(
        #     initial_pos,
        #     cur_pos,
        #     target_point, 
        #     self.distance_btw(cur_pos, target_point),
        #     self.dist_coef * self.distance_btw(cur_pos, target_point) + \
        #         du + self.speed_coef * np.math.fabs(fv - target_speed)
        # ))

        return cost

        # return self.dist_coef * self.distance_btw(cur_pos, target_point) + \
            # du + self.speed_coef * np.math.fabs(fv - target_speed)

    def distance_btw(self, cur_pos, target_point):
        p1 = np.array(cur_pos)
        p2 = np.array(target_point)
        return np.linalg.norm(p1 - p2)

    def angle_btw_points(self, p1, p2):
        return np.math.atan2(p2[1] - p1[1], p2[0] - p1[0])

    def calc_control(self, initial_pos, target_point, yaw, linear_speed, angular_speed, target_speed):
        # self.x0[0] = linear_speed
        # self.x0[1] = angular_speed
        solution = optimize.minimize(self.cost_function, 
                                     x0=self.x0,  
                                     bounds=self.bounds, 
                                     args=(initial_pos, target_point, yaw, linear_speed, angular_speed, target_speed),
                                     method='SLSQP',
                                     tol = 1e-8)#,
                                    #  options = {'disp': True})
        if solution.success:
            # print(solution.x)
            return solution.x[:2]

        return None


class Controller2:
    def __init__(self, horizon, dt, distance_threshold, dist_coef, lin_speed_coef, ang_speed_coef, speed_coef, dyaw_coef=1.0):
        self.horizon = horizon
        self.dt = dt
        self.distance_threshold = distance_threshold
        self.dist_coef = dist_coef
        self.lin_speed_coef = lin_speed_coef
        self.ang_speed_coef = ang_speed_coef
        self.speed_coef = speed_coef
        self.dyaw_coef = dyaw_coef

        self.x0 = [0.0] * self.horizon
        self.bounds = [[-0.50, 0.50] for _ in range(self.horizon)]

    def step(self, pos, yaw, linear_speed, angular_speed, dt):
        # a = (last_speed - linear_speed)
        x, y = pos
        x = x + linear_speed * np.math.cos(yaw) * dt
        y = y + linear_speed * np.math.sin(yaw) * dt
        yaw = yaw + angular_speed * dt
        return [x, y], yaw
    
    def find_theta(self, target_points, cur_pos):
        nearest_id = self.find_nearest_point_id(target_points, cur_pos)
        if nearest_id == target_points.shape[0] - 1:
            theta = self.compute_theta(target_points[nearest_id - 1], target_points[nearest_id])
        else:
            theta = self.compute_theta(target_points[nearest_id], target_points[nearest_id + 1])

        return theta, target_points[nearest_id]

    def compute_theta(self, p1, p2):
        return compute_theta(p1, p2)

    def find_nearest_point_id(self, target_points, cur_pos):
        u"""
        Args:
            target_points (numpy.ndarray)
            cur_pos (list)
        """
        cur_pos = np.array(cur_pos)

        idx = (np.linalg.norm(target_points - cur_pos, axis=1)).argmin()
        return idx

    def convert_orientation(self, theta):
        return convert_orientation(theta)

    def angles_difference(self, a1, a2):
        return angles_difference(a1, a2)

    def cost_function(self, u, initial_pos, target_points, yaw, linear_speed):
        cur_pos = [initial_pos[0], initial_pos[1]]
        cur_yaw = self.convert_orientation(yaw)

        lw = None

        cost = 0.0

        for i in range(self.horizon):
            w = u[i]
            cur_pos, cur_yaw = self.step(cur_pos, cur_yaw, linear_speed, w, self.dt)
            target_yaw, target_point = self.find_theta(target_points, cur_pos)
            cur_yaw = self.convert_orientation(cur_yaw)

            cost += (self.angles_difference(cur_yaw, target_yaw)) ** 2
            cost += (cur_pos[0] - target_point[0]) ** 2 + (cur_pos[1] - target_point[1]) ** 2 
            if lw is None:
                lw = w
            # cost += 0.1 * (lw - w) ** 2

        return cost

    def distance_btw(self, cur_pos, target_point):
        p1 = np.array(cur_pos)
        p2 = np.array(target_point)
        return np.linalg.norm(p1 - p2)

    def angle_btw_points(self, p1, p2):
        return np.math.atan2(p2[1] - p1[1], p2[0] - p1[0])

    def calc_control(self, initial_pos, target_points, yaw, linear_speed):
        # self.x0[0] = linear_speed
        # self.x0[1] = angular_speed
        solution = optimize.minimize(self.cost_function, 
                                     x0=self.x0,  
                                     bounds=self.bounds, 
                                     args=(initial_pos, target_points, yaw, linear_speed),
                                     method='SLSQP',
                                     tol = 1e-8,
                                     options = {'eps':0.1})
        if solution.success:
            # print(solution.x)
            return 0.0, solution.x[0]

        return None


class SpeedController(object):
    def __init__(self, speed_threshold=0.1, theta_threshold=0.1):
        self.theta_threshold = theta_threshold
        self.speed_threshold = speed_threshold

    def control_speed(self, initial_pos, yaw, current_speed, target_point, target_speed):
        theta = compute_theta(initial_pos, target_point)

        if abs(angles_difference(convert_orientation(yaw), theta)) > self.theta_threshold:
            print("BIGG DIFF BTWN ANGLES")
            return 0.0

        if abs(current_speed - target_speed) >  self.speed_threshold:
            coeffs = np.polyfit([0.0, 0.5, 1.0], [current_speed, 0.8 * target_speed, target_speed], 3)
            f = np.poly1d(coeffs)
            f2 = f(0.2)
            s = max(0.1, f2)
            s = min(target_speed, s)
            return s

        return target_speed

def main():
    node = ControllerNode('ControllerNode', 10)
    node.run()


if __name__ == '__main__':
    main()
