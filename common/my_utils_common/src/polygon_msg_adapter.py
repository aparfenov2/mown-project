# poligon message adapter
# https://github.com/kantengri/mown-project/blob/merge-branch/planning/enginx_msgs/msg/RouteTaskPolygon.msg

# std_msgs/Header header
# geometry_msgs/Polygon target_polygon

import rospy
import actionlib
from exploration_msgs.msg import ExploreAction, ExploreResult
from enginx_msgs.msg import RouteTaskPolygon

class Main:

    def __init__(self):
        rospy.init_node('polygon_msg_adapter')
        self._action_name = rospy.get_name()
        self._as = actionlib.SimpleActionServer(self._action_name, ExploreAction, execute_cb=self.execute_cb, auto_start = False)
        self.pub = rospy.Publisher('route_task_polygon', RouteTaskPolygon, queue_size=2)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo('%s: received ExploreAction' % self._action_name)
        pmsg = RouteTaskPolygon()
        pmsg.header = goal.boundary.header
        pmsg.target_polygon = goal.boundary.polygon
        self.pub.publish(pmsg)
        rospy.loginfo('%s: published RouteTaskPolygon')
        self._as.set_succeeded(ExploreResult())

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    Main().main()