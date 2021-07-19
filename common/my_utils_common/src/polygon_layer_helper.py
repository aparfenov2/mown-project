#!/usr/bin/env python  
import rospy
import actionlib
import exploration_msgs.msg
from exploration_msgs.srv import SetPolygon

class PolygonLayerHelper:
    # create messages that are used to publish feedback/result
    # _feedback = exploration_msgs.msg.FibonacciFeedback()
    _result = exploration_msgs.msg.ExploreResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, exploration_msgs.msg.ExploreAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.svc = rospy.ServiceProxy("move_base/global_costmap/polygon_layer/set_polygon", SetPolygon)
      
    def execute_cb(self, goal):
        rospy.loginfo('%s: Executing' % self._action_name)
        self.svc(goal.boundary)
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
    
if __name__ == '__main__':
    rospy.init_node('polygon_layer_helper')
    server = PolygonLayerHelper(rospy.get_name())
    rospy.spin()
