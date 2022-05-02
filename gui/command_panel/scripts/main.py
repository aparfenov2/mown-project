#!/usr/bin/env python3
import rospy
from rtree import index
from tkinter import Tk, Label, Button

from geometry_msgs.msg import PointStamped, PolygonStamped, Point32
from visualization_msgs.msg import Marker

from enginx_msgs.msg import RouteTaskToPoint, RouteTaskPolygon

STATE_NORMAL = 'normal'
STATE_DISABLE = 'disable'


class MainWindows(object):
    def __init__(self) -> None:
        self.window = Tk()
        lbl = Label(self.window, text="Command panel")
        lbl.grid(column=0, row=0)

        self.button_point_task = Button(
            self.window, text="Task Navigate To Point", command=self.button_point_task_callback)
        self.button_point_task.grid(column=0, row=1)

        self.button_publish_1 = Button(
            self.window, text="Publish", command=self.button_publish_1_callback)
        self.button_publish_1.grid(column=0, row=2)

        self.button_polygon_task = Button(
            self.window, text="Task Go Through Polygon", command=self.button_polygon_task_callback)
        self.button_polygon_task.grid(column=1, row=1)

        self.button_remove_all = Button(
            self.window, text="Remove All Points", command=self.button_remove_all_callback)
        self.button_remove_all.grid(column=1, row=2)

        self.button_publish_2 = Button(
            self.window, text="Publish", command=self.button_publish_2_callback)
        self.button_publish_2.grid(column=1, row=3)

        self.current_state = TaskPointState(self)

        rospy.Subscriber(
            '/clicked_point',
            PointStamped,
            self.clicked_point_callback
        )

    def run(self):
        self.window.mainloop()

    def set_current_state(self, new_state):
        self.current_state = new_state

    def button_point_task_callback(self):
        self.current_state.button_point_task_callback()

    def button_polygon_task_callback(self):
        self.current_state.button_polygon_task_callback()

    def button_publish_1_callback(self):
        self.current_state.button_publish_1_callback()

    def button_remove_all_callback(self):
        self.current_state.button_remove_all_callback()

    def button_publish_2_callback(self):
        self.current_state.button_publish_2_callback()

    def clicked_point_callback(self, message):
        self.current_state.clicked_point_callback(message)


class TaskPointState(object):
    def __init__(self, main_window) -> None:
        self.main = main_window
        self.main.button_point_task['state'] = STATE_DISABLE
        self.main.button_publish_1['state'] = STATE_NORMAL
        self.main.button_polygon_task['state'] = STATE_NORMAL
        self.main.button_remove_all['state'] = STATE_DISABLE
        self.main.button_publish_2['state'] = STATE_DISABLE

        self.pub = rospy.Publisher(rospy.get_param('/planner/topics/task_to_point_planning'), RouteTaskToPoint, queue_size=2)
        self.viz_publisher = rospy.Publisher('/debug/rviz/point', PointStamped, queue_size=2)

        self.message = RouteTaskToPoint()

    def button_point_task_callback(self):
        pass

    def button_polygon_task_callback(self):
        self.main.set_current_state(TaskPolygonState(self.main))

    def button_publish_1_callback(self):
        self.publish_task()

    def button_remove_all_callback(self):
        pass

    def button_publish_2_callback(self):
        pass

    def clicked_point_callback(self, message):
        self.message.target_pose.position.x = message.point.x
        self.message.target_pose.position.y = message.point.y
        self.publish()

    def publish(self):
        self.publish_visualization()

    def publish_task(self):
        self.message.header.stamp = rospy.get_rostime()
        self.pub.publish(self.message)

    def publish_visualization(self):
        point = PointStamped()
        point.header.stamp = rospy.get_rostime()
        point.header.frame_id = 'odom'
        point.point.x = self.message.target_pose.position.x
        point.point.y = self.message.target_pose.position.y
        self.viz_publisher.publish(point)


class TaskPolygonState(object):
    def __init__(self, main_window) -> None:
        self.main = main_window
        self.main.button_point_task['state'] = STATE_NORMAL
        self.main.button_publish_1['state'] = STATE_DISABLE
        self.main.button_polygon_task['state'] = STATE_DISABLE
        self.main.button_remove_all['state'] = STATE_NORMAL
        self.main.button_publish_2['state'] = STATE_NORMAL

        self.pub = rospy.Publisher(rospy.get_param('/planner/topics/task_polygon_planning'), RouteTaskPolygon, queue_size=2)
        self.viz_publisher = rospy.Publisher('/debug/rviz/polygon', PolygonStamped, queue_size=2)

        self.message = RouteTaskPolygon()
        self.visualized_poly = PolygonStamped()
        self.visualized_poly.header.frame_id = 'odom'

    def button_point_task_callback(self):
        self.main.set_current_state(TaskPointState(self.main))

    def button_polygon_task_callback(self):
        pass

    def button_publish_1_callback(self):
        pass

    def button_remove_all_callback(self):
        self.visualized_poly.polygon.points = list()
        self.publish_visualization()

    def button_publish_2_callback(self):
        self.publish_task()

    def clicked_point_callback(self, message):
        if len(self.visualized_poly.polygon.points) > 2:
            # idx = index.Index()
            # for ind, pt in enumerate(self.visualized_poly.polygon.points):
            #     idx.insert(ind, (pt.x, pt.y))

            # nearest_index_list = list(idx.nearest((message.point.x, message.point.y), 2))
            # print(nearest_index_list)
            # nearest_index = min(nearest_index_list)

            # if nearest_index == 0 and 1 not in nearest_index_list:
            #     nearest_index = max(nearest_index_list)

            new_point = Point32()
            new_point.x = message.point.x
            new_point.y = message.point.y

            # self.visualized_poly.polygon.points.insert(nearest_index + 1, new_point)
            self.visualized_poly.polygon.points.append(new_point)
        else:
            new_point = Point32()
            new_point.x = message.point.x
            new_point.y = message.point.y

            self.visualized_poly.polygon.points.append(new_point)

        self.publish_visualization()

    def publish(self):
        self.publish_visualization()

    def publish_task(self):
        if len(self.visualized_poly.polygon.points) > 2:
            self.message.header.stamp = rospy.get_rostime()
            # self.message.target_polygon = []
            # for point in self.visualized_poly:

            self.message.target_polygon = self.visualized_poly.polygon
            self.pub.publish(self.message)

    def publish_visualization(self):
        # if len(self.visualized_poly.polygon.points) > 0:
        self.visualized_poly.header.stamp = rospy.get_rostime()
        self.viz_publisher.publish(self.visualized_poly)


if __name__ == "__main__":
    rospy.init_node('command_panel_gui')
    MainWindows().run()
