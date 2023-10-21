# receive move_base_simple/goal
# submit query to move_base maske_plan service

#!/usr/bin/env python

import rospy
import tf
from nav_msgs.srv import GetPlan, GetPlanRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

def position_tuple_to_pose(x, y, yaw):
    """converts a position tuple to a geometry_msgs/Pose"""
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(*quat)
    position = Point(x, y, 0)
    return Pose(position, orientation)

class MapServer:
    def __init__(self):
        rospy.init_node('mb_service_client')
        self.current_pose = None
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.simple_goal_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.get_plan = rospy.ServiceProxy("move_base/make_plan", GetPlan)
        rospy.loginfo("Ready to add two ints.")

    def odom_callback(self, msg):
        self.frame_id = msg.header.frame_id
        self.current_pose = msg.pose.pose

    def simple_goal_cb(self, msg):
        rospy.logdebug("Accepted move_base_simple/goal msg " + str(msg))
        request = GetPlanRequest()
        request.start.header.stamp = rospy.Time.now()
        request.start.header.frame_id = self.frame_id
        request.start.pose = self.current_pose # position_tuple_to_pose(self.current_pose.position.x, self.current_pose.position.y, 0)

        request.goal.header.stamp = rospy.Time.now()
        request.goal.header.frame_id = self.frame_id
        request.goal.pose = Pose(Point(self.current_pose.position.x+0.1, self.current_pose.position.y+0.1, 0), self.current_pose.orientation)

        request.tolerance = 0.2

        rsp = self.get_plan(request)
        rospy.loginfo(f"Got new plan with {len(rsp.plan.poses)} points")

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    MapServer().main()
