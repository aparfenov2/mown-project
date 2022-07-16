import dubins
import numpy as np
import rospy
from task_behavior_engine.tree import Node, NodeStatus

from enginx_msgs.msg import Route, PointWithSpeed


class TestTrajectoryNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(TestTrajectoryNode, self).__init__(name=name,
                                                  run_cb=self.run,
                                                  *args, **kwargs)
        self._frame = frame
        self._published = False

    def run(self, nodedata):
        if (not self._frame.has_localization()):
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a localization")

        if not self._published:
            turning_radius = 3.0
            step_size = 0.1

            localization = self._frame.get_localization()
            x_0 = (localization.pose.x, localization.pose.y, localization.yaw)
            x_f = (localization.pose.x + 6, localization.pose.y + 2, np.deg2rad(100))

            path = dubins.shortest_path(x_0, x_f, turning_radius)
            configurations, _ = path.sample_many(step_size)

            route = Route()
            route.header.stamp = self._frame.get_localization().header.stamp
            for x, y, yaw in configurations:
                pws = PointWithSpeed()
                pws.x = x
                pws.y = y
                pws.speed = 0.8
                pws.d_time = 0.0
                route.route.append(pws)
            self._frame.set_trajectory(route)

            for i in range(1, 20):
                route.route[-i].speed = min(i * 0.01, route.route[i].speed)

            self._published = True
            print(f"Speed: {[r.speed for r in route.route]}")
        else:
            pass

        return NodeStatus(NodeStatus.SUCCESS)
