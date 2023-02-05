import rospy

from enginx_msgs.msg import CircleMovingTask, PlanningTaskType
from rosgraph_msgs.msg import Clock


def check_radius(radius):
    return radius is not None or radius > 0.0


def wait_for_clock():
    use_sim_time = False
    param_name = '/use_sim_time'

    if rospy.has_param(param_name):
        use_sim_time = rospy.get_param(param_name, False)

    if use_sim_time:
        rospy.logwarn("Wait for clock.")
        rospy.wait_for_message("/clock", Clock, timeout=None)


if __name__ == '__main__':
    left_radius = 3.0
    right_radius = 3.0
    target_speed = 0.5

    if not (check_radius(left_radius) or check_radius(right_radius)):
        exit()

    rospy.init_node('publish_circle_move_task', anonymous=True)

    wait_for_clock()

    task_type_publisher = rospy.Publisher(
        rospy.get_param('/planner/topics/behavior_planner/type_task'),
        PlanningTaskType,
        queue_size=10,
        latch=True
    )
    task_publisher = rospy.Publisher(
        rospy.get_param('/planner/topics/behavior_planner/circle_move_task'),
        CircleMovingTask,
        queue_size=10,
        latch=True
    )

    time_now = rospy.get_rostime()

    planning_task_type_message = PlanningTaskType()
    planning_task_type_message.header.stamp = time_now
    planning_task_type_message.type = PlanningTaskType.CIRCLE_MOVING_TASK
    planning_task_type_message.type = PlanningTaskType.CIRCLE_MOVING_TASK

    circle_move_task_message = CircleMovingTask()
    circle_move_task_message.header.stamp = time_now
    circle_move_task_message.target_speed = target_speed

    if check_radius(right_radius):
        circle_move_task_message.right_radius = right_radius

    if check_radius(left_radius):
        circle_move_task_message.left_radius = left_radius

    task_type_publisher.publish(planning_task_type_message)
    task_publisher.publish(circle_move_task_message)

    rospy.loginfo("Done...")
    rospy.spin()
