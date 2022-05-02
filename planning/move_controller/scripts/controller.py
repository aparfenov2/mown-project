#!/usr/bin/env python3
from math import cos
from threading import RLock

import rospy
import actionlib
import numpy as np
from scipy import optimize
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from abstractnode import AbstractNode
from move_controller import PIDController, NonlinearController, NonLinearMPC, Frame, PIDSpeedController, Target, SteerMPC, MoveMPC

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TwistStamped, Twist, Vector3
# from move_base_mod.msg import TrajectoryControllerAction, TrajectoryControllerGoal
from nav_msgs.msg import Odometry

from enginx_msgs.msg import LocalTrajectoryStamped, Localization, ControlDebug, Route
# from enginx_debug_msgs.msg import ControlDebug


def convert_orientation(theta):
    return np.math.atan2(np.sin(theta), np.cos(theta))


def angles_difference(a1, a2):
    # a = a2 - a1

    # if a > np.pi:
    #     a -= 2*np.pi
    # elif a < -np.pi:
    #     a += 2*np.pi
    
    return convert_orientation(a2-a1)


def compute_theta(p1, p2):
    return np.math.atan2(p2[1] - p1[1], p2[0] - p1[0])


class ControllerNode(AbstractNode):
    
    def initialization(self):
        self.__lock = RLock()
        self.__last_loc = None
        self.__last_trajectory = None

        controller_params = rospy.get_param('/planner/move_controller')

        self._frame = Frame()
        self._target = Target(self._frame, controller_params['target_distance'])

        self.nmpc = NonlinearController(self._frame)
        self.mpc = MoveMPC(self._frame, 1.0)
        self.speed_controller = PIDSpeedController(self._frame)
        self.speed_control = SpeedController(controller_params.get('speed_params', {}))

        self.steering_pid = PIDController(controller_params.get('steer_params', {}))
        self.mpc_steering = SteerMPC(self._frame)

        self.control_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/velocity_commands'),
            Twist,
            queue_size=10
        )

        # self.control_debug_publisher = rospy.Publisher(
        #     'planner/debug/control',
        #     ControlDebug,
        #     queue_size=10
        # )

        self._debug_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/debug/control'),
            ControlDebug,
            queue_size=10
        )

        self.last_linear_speed = 0
        self.last_angular_speed = 0

        self.speed = 0.0
        self.last_idx = -1

        # rospy.Subscriber('/local_trajectory_plan', LocalTrajectoryStamped, self.__route_callback)
        rospy.Subscriber(rospy.get_param('/planner/topics/route/control'),
                         Route,
                         self._frame.receive_route)
        # rospy.Subscriber('/laser_odom_to_init', Odometry, self.__odometry_callback)
        rospy.Subscriber(rospy.get_param('/planner/topics/localization'),
                         Localization,
                         self._frame.receive_localization)

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
        with self._frame.lock():
            self._frame.reset_debug()

            if not self._frame.has_localization() or not self._frame.has_trajectory():
                self.send_idle_control()
                return

            self._target.prepare_data()

            if self.__close_to_end():
                self.send_idle_control()
                self.last_idx = -1
                return

            # target_point = self.__get_target_point(current_pos)

            # print("Cur: {0}, target: {1}".format(
            #     current_pos, target_point
            # ))
            # linear_velocity = self.__linear_velocity_control(current_pos, target_point)
            # angular_velocity = self.__angular_velocity_control(current_pos, target_point)

            # linear_velocity, angular_velocity = self.mpc_control(
            #     [current_pos[0], current_pos[1]],
            #     [target_point[0], target_point[1]],
            #     current_pos[2],
            #     target_point[2]
            # )
            # linear_velocity = self.speed_controller.execute(self._target)
            # angular_velocity = self.mpc_steering.execute(self._target)
            # linear_velocity, angular_velocity = self.mpc.execute(self._target)
            linear_velocity, angular_velocity = self.nmpc.execute()

            if linear_velocity is None:
                linear_velocity = 0.0
            # angular_velocity = self.__calculate_angular_velocity()
            print("CALCULATE: ", linear_velocity, angular_velocity)

            if angular_velocity is None:
                angular_velocity = 0.0
            self.send_control(linear_velocity, angular_velocity)

    def __close_to_end(self):
        pos = self._frame.get_robot_location()
        last_point = self._frame.get_trajectory_as_list()[-1][:2]
        p1 = np.array(pos)
        p2 = np.array(last_point)
        return np.linalg.norm(p1 - p2) < 0.2

    def __calculate_angular_velocity(self):
        current_location = self._frame.get_robot_location()
        target_location = self._target.get_target_point()
        yaw = self._frame.get_robot_yaw()

        theta = compute_theta(current_location, target_location)
        dw = angles_difference(convert_orientation(yaw), theta)
        w = self.steering_pid.execute(dw)
        return w

    def mpc_control(self, initial_pos, target_point, yaw, target_speed):
        path = np.array([p[:2] for p in self.__last_trajectory])
        # res = self.mpc.calc_control(initial_pos, path, yaw, self.speed)
        # res = self.mpc.calc_control(initial_pos, target_point, yaw, self.speed, target_speed)
        speed = self.speed_control.control_speed(initial_pos, yaw, self.speed, target_point, target_speed)

        # print('Optim control: {}'.format(res))

        theta = compute_theta(initial_pos, target_point)
        dw = angles_difference(convert_orientation(yaw), theta)
        w = self.steering_pid.execute(dw)

        print("Calculated: speed - {}, ang. rate - {}".format(
            speed, w
        ))

        # if res is None:
        #     v, w = 0, dw * 0.1
        #     rospy.logerr('No optimal control')
        # else:
        #     v, w = res

        # if abs(dw) > 0.1:
        #     v = 0.0

        v = speed

        # print("Control: {0}, {1}".format(v, w))

        linear_vel = Vector3(x=v, y=0.0, z=0.0)
        angular_vel = Vector3(x=0.0, y=0.0, z=w)
        self.last_linear_speed = v
        self.last_angular_speed = w
        return linear_vel, angular_vel      

    def send_control(self, linear_velocity, angular_velocity):
        message = Twist()
        message.linear.x = linear_velocity
        message.angular.z = angular_velocity
        self.control_publisher.publish(message)

        # debug_msg = ControlDebug()
        # debug_msg.linear_velocity = linear_velocity
        # debug_msg.angular_velocity = angular_velocity
        # debug_msg.header.stamp = rospy.get_rostime()

        # self.control_debug_publisher.publish(debug_msg)
        self._debug_publisher.publish(self._frame.control_debug)

    def send_idle_control(self):
        self.send_control(0.0, 0.0)

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

        self.x0 = np.array([0.0] * (2 * self.horizon))
        self.bounds = list()
        for _ in range(self.horizon):
            self.bounds += [[0.0, 0.50]]
            self.bounds += [[-0.10, 0.10]]

    def step(self, pos, yaw, linear_speed, angular_speed, dt):
        # a = (last_speed - linear_speed)
        x, y = pos
        x = x + linear_speed * np.math.cos(yaw) * dt
        y = y + linear_speed * np.math.sin(yaw) * dt
        yaw = yaw + angular_speed * dt
        return [x, y], yaw

    def cost_function(self, u, initial_pos, target_point, yaw, linear_speed, target_speed):
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
            # cost += (v - target_speed) ** 2
            
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

    def calc_control(self, initial_pos, target_point, yaw, linear_speed, target_speed):
        # self.x0[0] = linear_speed
        # self.x0[1] = angular_speed

        self.x0 = np.delete(self.x0, 0)
        self.x0 = np.delete(self.x0, 0)
        self.x0 = np.append(self.x0, self.x0[-2])
        self.x0 = np.append(self.x0, self.x0[-2])

        solution = optimize.minimize(self.cost_function, 
                                     x0=self.x0,  
                                     bounds=self.bounds, 
                                     args=(initial_pos, target_point, yaw, linear_speed, target_speed),
                                     method='SLSQP',
                                     tol = 1e-8)#,
                                    #  options = {'disp': True})
        if solution.success:
            # print(solution.x)
            self.x0 = solution.x
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

        self.x0 = np.array([0.10] * self.horizon)
        self.bounds = [[-0.10, 0.10] for _ in range(self.horizon)]

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
        ntp = target_points[idx:]
        idx = (np.linalg.norm(ntp - cur_pos) - 0.5).argmin()

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

        if linear_speed == 0.0:
            linear_speed = 0.1

        for i in range(self.horizon):
            w = u[i]
            cur_pos, cur_yaw = self.step(cur_pos, cur_yaw, linear_speed, w, self.dt)
            target_yaw, target_point = self.find_theta(target_points, cur_pos)
            cur_yaw = convert_orientation(cur_yaw)

            # print("Target pos: {0}, target yaw: {1}, cur_pos: {2}, cur_yaw: {3}, cost: {4}".format(
            #     target_point, target_yaw, cur_pos, cur_yaw, abs(self.angles_difference(cur_yaw, target_yaw))
            # ))

            ttheta = compute_theta(cur_pos, target_point)
            # cost += abs(self.angles_difference(cur_yaw, ttheta))
            # print('Cost btw poits: {0}, cur_yaw: {1}, target yaw: {2}'.format(
            #         self.angles_difference(cur_yaw, target_yaw),
            #         cur_yaw,
            #         target_yaw
            #     ))
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

        self.x0 = np.delete(self.x0, 0)
        self.x0 = np.delete(self.x0, 0)
        self.x0 = np.append(self.x0, self.x0[-2])
        self.x0 = np.append(self.x0, self.x0[-2])

        solution = optimize.minimize(self.cost_function, 
                                     x0=self.x0,  
                                     bounds=self.bounds, 
                                     args=(initial_pos, target_points, yaw, linear_speed),
                                     method='SLSQP',
                                     tol = 1e-8,
                                     options = {'eps': 0.01, 'disp': True})
        if solution.success:
            # print(solution.x)
            self.x0[1:] = self.x0[0: -1]
            self.x0[0] = solution.x[0]
            return 0.0, solution.x[0]

        return None


class SpeedController(object):
    def __init__(self, controller_params):
        self.theta_threshold = controller_params.get('theta_threshold', 0.1)

        self.speed_pid = PIDController(controller_params)

        self.debug_publisher = rospy.Publisher(
            'planner/debug/ang_dif',
            Float32, 
            queue_size=10
        )

    def control_speed(self, initial_pos, yaw, current_speed, target_point, target_speed):
        theta = compute_theta(initial_pos, target_point)

        if abs(angles_difference(yaw, theta)) > self.theta_threshold:
            print("BIGG DIFF BTWN ANGLES: {0}, yaw: {1}, theta: {2}".format(
                angles_difference(yaw, theta), yaw, theta
            ))
            s = 0.0
        else:
            s = self.speed_pid.execute(target_speed - current_speed)

        msg = Float32()
        msg.data = abs(angles_difference(yaw, theta))
        self.debug_publisher.publish(msg)

        return s
            

         
        # theta = compute_theta(initial_pos, target_point)

        # if abs(angles_difference(yaw, theta)) > self.theta_threshold:
        #     print("BIGG DIFF BTWN ANGLES: {0}, yaw: {1}, theta: {2}".format(
        #         angles_difference(yaw, theta), yaw, theta
        #     ))
        #     return 0.0

        # if abs(current_speed - target_speed) > self.speed_threshold:
        #     print("DIFF BTWN ANGLES: {0}, yaw: {1}, theta: {2}".format(
        #         angles_difference(yaw, theta), yaw, theta
        #     ))
        #     coeffs = np.polyfit([0.0, 0.5, 1.0], 
        #                         [current_speed, 
        #                          0.5 * current_speed * (target_speed - current_speed), 
        #                          target_speed], 
        #                         1)
        #     f = np.poly1d(coeffs)
        #     f2 = f(0.2)
        #     s = max(0.1, f2)
        #     s = min(target_speed, s)
        #     return s

        # return target_speed

def main():
    node = ControllerNode('ControllerNode', 10)
    node.run()


if __name__ == '__main__':
    main()
