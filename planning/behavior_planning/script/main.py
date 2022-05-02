#!/usr/bin/env python3

import rospy
import threading
from task_behavior_engine.branch import Sequencer

from abstractnode import AbstractNode
from behavior_planning.common import Frame
from behavior_planning.node import (TrajectoryPublisher,
                                    AstarPathPlanningNode,
                                    IdleNode, SpeedGeneratorNode,
                                    CovaragePathGeneratorNode)
from behavior_planning.behavior import SelectByMessageNode

from nav_msgs.msg import OccupancyGrid
from enginx_msgs.msg import (Route, Localization, RouteTaskPolygon,
                             RouteTaskToPoint, PlanningDebug)


class BehaviorPlanningNode(AbstractNode):

    def initialization(self):
        self._rlock = threading.RLock()
        self._frame = Frame()
        self._planner = BehaviorPlanner(self._frame)

        self._trajectory_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/route/control'),
            Route,
            queue_size=10
        )

        self._debug_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/debug/planning'),
            PlanningDebug,
            queue_size=10
        )

        rospy.Subscriber(rospy.get_param('/planner/topics/task_polygon_planning'),
                         RouteTaskPolygon,
                         self.receive_task_polygon_planning)
        rospy.Subscriber(rospy.get_param('/planner/topics/task_to_point_planning'),
                         RouteTaskToPoint,
                         self.receive_task_to_point_planning)
        rospy.Subscriber(rospy.get_param('/planner/topics/costmap'),
                         OccupancyGrid, self.receive_occupancy_grid_map)
        rospy.Subscriber(rospy.get_param('/planner/topics/localization'),
                         Localization,
                         self.receive_localization)

    def work(self):
        with self._rlock:
            self._frame.reset_trajectory()
            self._planner.execute()
            self._debug_publisher.publish(self._frame.planning_debug)
            trajectory = self._frame.trajectory
            self._trajectory_publisher.publish(trajectory)

    def receive_task_polygon_planning(self, message):
        with self._rlock:
            self._frame.reset_route_task_to_point_message()
            self._frame.set_route_task_polygon_message(message)

    def receive_task_to_point_planning(self, message):
        with self._rlock:
            self._frame.reset_route_task_polygon_message()
            self._frame.set_route_task_to_point_message(message)

    def receive_localization(self, message):
        with self._rlock:
            self._frame.set_localization(message)

    def receive_occupancy_grid_map(self, message):
        with self._rlock:
            self._frame.receive_map_message(message)


class BehaviorTreeBuilder(object):
    def build(self, frame):
        planning_node = AstarPathPlanningNode(name="planning_node",
                                              frame=frame)
        speed_generator_node = SpeedGeneratorNode(
            name='speed_generator_node',
            frame=frame)
        idle_node = IdleNode(name='idle_node', frame=frame)
        coverage_node = CovaragePathGeneratorNode(
            name='coverage_path_node',
            frame=frame)

        planning_branche = Sequencer("finish_counts")
        planning_branche.add_child(planning_node)
        planning_branche.add_child(speed_generator_node)

        coverage_branch = Sequencer("coverage_branch")
        coverage_branch.add_child(coverage_node)

        idle_branche = Sequencer("idle_branch")
        idle_branche.add_child(idle_node)

        behavior_selector = SelectByMessageNode("behavior_selector",
                                                frame,
                                                idle_branche,
                                                planning_branche,
                                                coverage_branch)
        return behavior_selector


class BehaviorPlanner(object):
    def __init__(self, frame) -> None:
        self._frame = frame
        self._tree = BehaviorTreeBuilder().build(self._frame)

    def execute(self):
        return self._tree.tick()


if __name__ == '__main__':
    BehaviorPlanningNode('BehaviorPlanningNode', 5).run()
