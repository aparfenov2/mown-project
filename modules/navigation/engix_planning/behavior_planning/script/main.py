#!/usr/bin/env python3

from pathlib import Path
import rospy
import threading
from task_behavior_engine.branch import Sequencer

from abstractnode import AbstractNode
from behavior_planning.common import Frame
from behavior_planning.node import (TrajectoryPublisher,
                                    AstarPathPlanningNode,
                                    IdleNode, SpeedGeneratorNode,
                                    CovaragePathGeneratorNode, CoverageNode,
                                    TestTrajectoryNode, SimpleSpeedGenerator,
                                    TestStraightLineNode, TestCirclesNode,
                                    CircleMovingNode, LineMovingNode,
                                    DubingPlanningNode, CoveragePlanPathGeneratorNode)
from behavior_planning.behavior import SelectByMessageNode, TaskMessageSelector

from nav_msgs.msg import OccupancyGrid, Path
from engix_msgs.msg import (Route, Localization, RouteTaskPolygon,
                             RouteTaskToPoint, PlanningDebug,
                             LineMovingTask, PlanningTaskType,
                             CoveragePlanningTask, CircleMovingTask)


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

        self._debug_coverage_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/debug/coverage_path'),
            Path,
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
        rospy.Subscriber(rospy.get_param('/planner/topics/behavior_planner/type_task'),
                         PlanningTaskType,
                         self._frame.planning_task_type.receive_message)
        rospy.Subscriber(rospy.get_param('/planner/topics/behavior_planner/line_move_task'),
                         LineMovingTask,
                         self._frame.line_moving_task.receive_message)
        rospy.Subscriber(rospy.get_param('/planner/topics/behavior_planner/circle_move_task'),
                         CircleMovingTask,
                         self._frame.circle_moving_task.receive_message)
        rospy.Subscriber(rospy.get_param('/planner/topics/behavior_planner/coverage_planning_task'),
                         CoveragePlanningTask,
                         self._frame.coverage_planning_task.receive_message)

    def work(self):
        with self._rlock:
            self._frame.reset_trajectory()
            self._planner.execute()
            self._debug_publisher.publish(self._frame.planning_debug)
            self.debug_publish()
            if self._frame.has_trajectory():
                trajectory = self._frame.trajectory
                self._trajectory_publisher.publish(trajectory)

    def debug_publish(self):
        path = self._frame.debug_data.coverage_path

        if path:
            self._debug_coverage_publisher.publish(path)

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
        params = rospy.get_param('/planner/behavior_planning_node')
        planning_node = AstarPathPlanningNode(name="planning_node",
                                              frame=frame)
        speed_generator_node = SimpleSpeedGenerator(
            name='speed_generator_node',
            frame=frame
        )
        idle_node = IdleNode(name='idle_node', frame=frame)
        coverage_node = CoverageNode(
            name='coverage_path_node',
            frame=frame,
            params=params
        )

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


class MessageSelectionBehaviorTreeBuilder(object):
    def build(self, frame):
        idle_node = IdleNode(name='idle_node', frame=frame)
        line_move_node = LineMovingNode(name='line_move_node', frame=frame)
        circle_move_node = CircleMovingNode(name='circle_move_node', frame=frame)
        dubins_planning_node = DubingPlanningNode(name='dubins_planning_node', frame=frame)
        coverage_planning_node = CoveragePlanPathGeneratorNode(name='coverage_planning_node', frame=frame)

        node_selector = TaskMessageSelector(name='idle_node',
                                            frame=frame,
                                            default_node=idle_node,
                                            line_moving_node=line_move_node,
                                            circle_moving_node=circle_move_node,
                                            dubins_planning_node=dubins_planning_node,
                                            coverage_node=coverage_planning_node)
        return node_selector


class TestBTreeBuilder(object):
    def build(self, frame):
        # test_node = TestCirclesNode(name="test_node",
        #                             target_speed=0.6,
        #                             left_distance=6.0,
        #                             frame=frame)
        test_node = TestStraightLineNode(name="test_node",
                                         line_len=5.0,
                                         frame=frame)

        planning_branche = Sequencer("finish_counts")
        planning_branche.add_child(test_node)
        return planning_branche


class BehaviorPlanner(object):
    def __init__(self, frame) -> None:
        self._frame = frame
        # self._tree = BehaviorTreeBuilder().build(self._frame)
        # self._tree = TestBTreeBuilder().build(self._frame)
        self._tree = MessageSelectionBehaviorTreeBuilder().build(self._frame)

    def execute(self):
        return self._tree.tick()


if __name__ == '__main__':
    BehaviorPlanningNode('BehaviorPlanningNode', 5).run()
