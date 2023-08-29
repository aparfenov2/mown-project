#!/usr/bin/env python3

import threading
import rospy
import tf
import argparse

from geometry_msgs.msg import PoseStamped
from engix_msgs.msg import DubinsPlanningTask


class BridgeNode:
    def __init__(
        self,
        target_speed,
        turning_radius,
        step_size,
        publish_topic: str = '/global_planner/dubins_planning_task',
        subscribe_topic: str = '/move_base_simple/goal'

    ):
        self.dubins_task = DubinsPlanningTask()
        self.dubins_task.target_speed = target_speed
        self.dubins_task.turning_radius = turning_radius
        self.dubins_task.step_size = step_size

        self.task_publisher = rospy.Publisher(
            publish_topic,
            DubinsPlanningTask,
            queue_size=10
        )

        rospy.Subscriber(subscribe_topic,
                         PoseStamped,
                         self.goal_callback)

    def goal_callback(self, message):
        position = message.pose.position
        quaternion = (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w
        )

        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.dubins_task.header = message.header
        self.dubins_task.target_pose.x = position.x
        self.dubins_task.target_pose.y = position.y
        self.dubins_task.target_pose.theta = yaw

        self.task_publisher.publish(self.dubins_task)


def parse_args():
    parser = argparse.ArgumentParser(description='Train segmentation network')

    parser.add_argument('--target_speed',
                        help='experiment configure file name',
                        default=0.1,
                        type=float)

    parser.add_argument('--turning_radius',
                        help='experiment configure file name',
                        default=2.0,
                        type=float)

    parser.add_argument('--step_size',
                        help='experiment configure file name',
                        default=0.1,
                        type=float)

    args, unknown = parser.parse_known_args()

    return args


if __name__ == '__main__':
    rospy.init_node('point_to_dubins_node', anonymous=True)
    args = parse_args()
    node = BridgeNode(
        args.target_speed,
        args.turning_radius,
        args.step_size,
    )
    rospy.spin()
