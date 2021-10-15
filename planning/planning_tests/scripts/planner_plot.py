#!/usr/bin/env python3


import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import threading

import numpy as np
import rospy

from enginx_msgs.msg import LocalTrajectoryStamped, Localization, Route, RouteTaskToPoint


class PlotNode(object):
    def __init__(self):
        self.rlock = threading.RLock() 

        self.robot_pose_plotter = RobotPosePlotter()    
        self.path_plotter = PathPlotter()
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

    def main(self):
        self.fig, self.ax = plt.subplots()
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.line, = self.ax.plot([], [])
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


    def __route_task_callback(self, msg):
        with self.rlock:
            self.path_plotter.receive(msg)

    def __odometry_callback(self, msg):
        with self.rlock:
            self.robot_pose_plotter.receive(msg)

    def init(self):
        self.ax.set_xlim(0, 2*np.pi)
        self.ax.set_ylim(-1, 1)
        return ln,

    def update(self, frame):
        ret = []
        data = self.robot_pose_plotter.get_data()
        # self.ax.p
        sd = self.ax.add_patch(data)
        ret.append(sd)

        xs, ys = self.path_plotter.get_data()

        if(len(xs) > 0):
            self.line.set_data(xs, ys)
            ret.append(self.line)
        return ret


class RobotPosePlotter(object):
    def __init__(self, width=1.0, height=2.0):
        self.x, self.y = 0.0, 0.0
        self.yaw = 0.0
        self.width, self.height = width, height
        self.p = None

    def receive(self, msg):
        self.x = msg.pose.x 
        self.y = msg.pose.y
        self.yaw =  msg.yaw

    def get_data(self):   
        if self.p is not None:
            self.p.remove()

        # self.p = patches.Rectangle((self.x, self.y), self.width, self.height, angle=np.degrees(self.yaw), edgecolor="red")
        self.p = plt.Circle((self.x, self.y), 0.5, color='b', fill=False)
        return self.p


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