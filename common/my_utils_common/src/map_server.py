#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import OccupancyGrid
from occupancy_grid_python import OccupancyGridManager
import ros_numpy

class MapServer:
    def __init__(self):
        rospy.init_node('map_server')
        self.ogm = OccupancyGridManager('/map', subscribe_to_updates=True)  # default False
        rospy.Service('static_map', GetMap, self.handle_get_map)
        rospy.loginfo("Ready to publish occupancy grid as service")

    def handle_get_map(self, req):
        rospy.loginfo('handle_get_map: map returned')
        msg = ros_numpy.msgify(OccupancyGrid, self.ogm._grid_data, self.ogm._occ_grid_metadata)
        return GetMapResponse(msg)

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    MapServer().main()
