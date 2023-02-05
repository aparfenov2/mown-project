import rospy

from geometry_msgs.msg import PoseStamped
from enginx_msgs.msg import CoveragePlanningTask, PlanningTaskType
from rosgraph_msgs.msg import Clock


def wait_for_clock():
    use_sim_time = False
    param_name = '/use_sim_time'

    if rospy.has_param(param_name):
        use_sim_time = rospy.get_param(param_name, False)

    if use_sim_time:
        rospy.wait_for_message("/clock", Clock, timeout=None)


if __name__ == '__main__':
    turning_radius = 3.0
    target_speed = 0.2

    rospy.init_node('publish_line_move_task', anonymous=True)

    wait_for_clock()

    task_type_publisher = rospy.Publisher(
        rospy.get_param('/planner/topics/behavior_planner/type_task'),
        PlanningTaskType,
        queue_size=1,
        latch=True
    )
    task_publisher = rospy.Publisher(
        rospy.get_param('/planner/topics/behavior_planner/coverage_planning_task'),
        CoveragePlanningTask,
        queue_size=1,
        latch=True
    )

    time_now = rospy.Time.now()

    planning_task_type_message = PlanningTaskType()
    planning_task_type_message.header.stamp = time_now
    planning_task_type_message.type = PlanningTaskType.COVERAGE_TASK

    coverage_task_message = CoveragePlanningTask()
    coverage_task_message.header.stamp = time_now
    coverage_task_message.turning_radius = turning_radius
    coverage_task_message.step_size = 0.1
    coverage_task_message.target_speed = target_speed

    for point in [(1.0, 0.0), (8.0, 0.0), (7.0, 7.0), (1.0, 7.0)]:
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        coverage_task_message.path.poses.append(pose)

    task_type_publisher.publish(planning_task_type_message)
    task_publisher.publish(coverage_task_message)

    rospy.loginfo("Done...")
    rospy.spin()
