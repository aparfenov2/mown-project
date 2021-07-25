#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import OccupancyGrid

class MapServer:
    def __init__(self):
        self.map = None

    def grid_callback(self, data):
        self.map = data

    def handle_get_map(self, req):
        assert self.map is not None
        rospy.loginfo('handle_get_map: map returned')
        return GetMapResponse(self.map)

    def main(self):
        rospy.init_node('map_server')
        rospy.Service('static_map', GetMap, self.handle_get_map)
        rospy.Subscriber("/map", OccupancyGrid, self.grid_callback)
        rospy.loginfo("Ready to add two ints.")
        rospy.spin()

if __name__ == "__main__":
    MapServer().main()
