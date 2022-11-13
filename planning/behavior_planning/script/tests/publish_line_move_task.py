import rospy

from enginx_msgs.msg import LineMovingTask, PlanningTaskType
from rosgraph_msgs.msg import Clock


def wait_for_clock():
    use_sim_time = False
    param_name = '/use_sim_time'

    if rospy.has_param(param_name):
        use_sim_time = rospy.get_param(param_name, False)

    if use_sim_time:
        rospy.wait_for_message("/clock", Clock, timeout=None)


if __name__ == '__main__':
    distance = 5.0
    target_speed = 0.5

    rospy.init_node('publish_line_move_task', anonymous=True)

    wait_for_clock()

    task_type_publisher = rospy.Publisher(
        rospy.get_param('/planner/topics/behavior_planner/type_task'),
        PlanningTaskType,
        queue_size=1,
        latch=True
    )
    task_publisher = rospy.Publisher(
        rospy.get_param('/planner/topics/behavior_planner/line_move_task'),
        LineMovingTask,
        queue_size=1,
        latch=True
    )

    time_now = rospy.Time.now()
    # time_now = rospy.get_rostime()

    print(f"{time_now}")
    planning_task_type_message = PlanningTaskType()
    planning_task_type_message.header.stamp = time_now
    planning_task_type_message.type = PlanningTaskType.LINE_MOVING_TASK

    line_move_task_message = LineMovingTask()
    line_move_task_message.header.stamp = time_now
    line_move_task_message.distance = distance
    line_move_task_message.target_speed = target_speed

    task_type_publisher.publish(planning_task_type_message)
    task_publisher.publish(line_move_task_message)

    rospy.loginfo("Done...")
    rospy.spin()
