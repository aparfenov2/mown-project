import rospy
from task_behavior_engine.tree import Node, NodeStatus

from engix_msgs.msg import Route


class TrajectoryPublisher(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(TrajectoryPublisher, self).__init__(name=name,
                                                  run_cb=self.run,
                                                  *args, **kwargs)
        self._frame = frame
        self._trajectory_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/route/control'),
            Route,
            queue_size=10
        )

    def run(self, nodedata):
        if (not self._frame.has_trajectory()):
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a trajectory")

        trajectory = self._frame.trajectory
        self._trajectory_publisher.publish(trajectory)
        return NodeStatus(NodeStatus.SUCCESS)
