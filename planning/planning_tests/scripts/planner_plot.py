#!/usr/bin/env python3


import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import threading

import numpy as np
import rospy
from tf import TransformListener

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path, OccupancyGrid
from enginx_msgs.msg import LocalTrajectoryStamped, Localization, Route, RouteTaskToPoint


class PlotNode(object):
    def __init__(self):
        self.rlock = threading.RLock() 

        self.robot_pose_plotter = RobotPosePlotter()    
        self.path_plotter = PathPlotter()
        self.grid_plotter = GridMapPlotter()

        self.pub = rospy.Publisher(
            rospy.get_param('/planner/topics/task_to_point_planning'), 
            RouteTaskToPoint, 
            queue_size=2
        )
        
        rospy.Subscriber(rospy.get_param('/planner/topics/route'), 
                         Route, 
                         self.__route_task_callback)
        rospy.Subscriber(rospy.get_param('/planner/topics/localization'), 
                         Localization, 
                         self.__odometry_callback)
        rospy.Subscriber(
            rospy.get_param('/planner/topics/costmap'),
            OccupancyGrid, 
            self.__occupancy_grid_map_callback
        )

    def main(self):
        self.fig, self.ax = plt.subplots()
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.line, = self.ax.plot([], [])
        self.head_line, = self.ax.plot([], [])
        self.pc, = self.ax.plot([], [], 'go')
        ani = FuncAnimation(self.fig, self.update)
        plt.show()

    def onclick(self, event):
        print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          ('double' if event.dblclick else 'single', event.button,
           event.x, event.y, event.xdata, event.ydata))
        if event.dblclick:
            message = RouteTaskToPoint()
            message.header.stamp = rospy.get_rostime()
            message.target_pose.position.x = event.xdata
            message.target_pose.position.y = event.ydata
            self.pub.publish(message)

    def __occupancy_grid_map_callback(self, msg):
        with self.rlock:
            self.grid_plotter.receive(msg)

    def __route_task_callback(self, msg):
        with self.rlock:
            self.path_plotter.receive(msg)

    def __odometry_callback(self, msg):
        with self.rlock:
            self.robot_pose_plotter.receive(msg)
            self.grid_plotter.receive_rob_pos(msg)

    def init(self):
        self.ax.set_xlim(0, 2*np.pi)
        self.ax.set_ylim(-1, 1)
        return ln,

    def update(self, frame):
        ret = []
        data, head_data = self.robot_pose_plotter.get_data()
        # self.ax.p
        sd = self.ax.add_patch(data)
        ret.append(sd)

        self.head_line.set_data(head_data[0], head_data[1])
        ret.append(self.head_line)

        xs, ys = self.path_plotter.get_data()


        if(len(xs) > 0):
            self.line.set_data(xs, ys)
            ret.append(self.line)

        pc_x, pc_y = self.grid_plotter.get_points()
        if len(pc_x) > 0:
            self.pc.set_data(pc_x, pc_y)
            ret.append(self.pc)

        return ret


class RobotPosePlotter(object):
    def __init__(self, width=1.0, height=2.0, radius=0.5):
        self.x, self.y = 0.0, 0.0
        self.yaw = 0.0
        self.width, self.height = width, height
        self.p = None

        self.radius = radius

    def receive(self, msg):
        self.x = msg.pose.x 
        self.y = msg.pose.y
        self.yaw =  msg.yaw

    def get_data(self):   
        if self.p is not None:
            self.p.remove()

        # self.p = patches.Rectangle((self.x, self.y), self.width, self.height, angle=np.degrees(self.yaw), edgecolor="red")
        self.p = plt.Circle((self.x, self.y), self.radius, color='b', fill=False)
        p1, p2 = self.calc_heading()
        return self.p, [[p1[0], p2[0]], [p1[1], p2[1]]]

    def calc_heading(self):
        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        p = np.array([
            c * self.radius, s * self.radius
        ])

        p_rob = np.array([self.x, self.y])
        p_start = np.array([0.0, 0.0])

        return p_start + p_rob, p + p_rob


class PathPlotter(object):
    def __init__(self):
        self.xs, self.ys = [], []

    def receive_polygon(self, msg):

        self.xs, self.ys = [], []
        
        for point in msg.route:
            self.xs.append(point.x)
            self.ys.append(point.y)

    def receive_path(self, msg):
    
        self.xs, self.ys = [], []
        
        for point in msg.target_pose:
            self.xs.append(point.x)
            self.ys.append(point.y)

    def receive(self, msg):
        self.xs = []
        self.ys = []

        for point in msg.route:
            self.xs.append(point.x)
            self.ys.append(point.y)


    def get_data(self):
        return self.xs, self.ys


class GridMapPlotter(object):
    def __init__(self) -> None:
        super().__init__()
        self.pc_x, self.pc_y = [], []
        self.rob_pos = np.array([0.0, 0.0])
        self.yaw = 0.0

        # Initialize the listener (needs some time to subscribe internally to TF and fill its buffer)
        self.tl = TransformListener()

    def receive_rob_pos(self, msg):
        self.rob_pos = np.array([msg.pose.x, msg.pose.y])
        self.yaw = msg.yaw

    def convert_pose(self, pose):# Our point would look like this
        p = PointStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'velodyne'
        p.point.x = 1.0
        p.point.y = 0.5
        p.point.z = 0.0

        # Transform the point from base_footprint to map
        map_p = self.tl.asMatrix('odom', p)

        
    def receive(self, message):
        points = []
        # xs, ys = [], []
        
        resolution = message.info.resolution
        pos_x = message.info.origin.position.x
        pos_y = message.info.origin.position.y
        width = message.info.width
        height = message.info.height

        print('Help text', pos_x, pos_y, message.info.origin.orientation, message.header)

        # c, s = np.cos(self.yaw), np.sin(self.yaw)
        # # R = np.array(((c, -s), (s, c)))
        # message.header.frame_id = 'velodyne'
        # R = self.tl.asMatrix('world', message.header)

        for x in range(width):
            for y in range(height):
                if message.data[x + y * width] > 0:
                    world_x = (x * resolution) + pos_x + resolution / 2
                    world_y = (y * resolution) + pos_y + resolution / 2

                    # self.points.append([world_x, world_y])
                    # xs.append(world_x)
                    # ys.append(world_y)
                    v = np.array([world_x, world_y])
                    points.append(v)

        points = np.array(points)
        print('points shape', points.shape)
        self.pc_x = points[:, 0]
        self.pc_y = points[:, 1]

    def get_points(self):
        return self.pc_x, self.pc_y



# fig, ax = plt.subplots()
# xdata, ydata = [], []
# ln, = plt.plot([], [], 'ro')

# def init():
#     ax.set_xlim(0, 2*np.pi)
#     ax.set_ylim(-1, 1)
#     return ln,

# def update(frame):
#     xdata.append(frame)
#     ydata.append(np.sin(frame))
#     ln.set_data(xdata, ydata)
#     return ln,

# ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
#                     init_func=init, blit=True)
# plt.show()


if __name__ == '__main__':
    rospy.init_node('robot_plot')
    node = PlotNode()
    node.main()