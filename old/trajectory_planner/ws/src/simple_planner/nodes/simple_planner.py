import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_mod.msg import TrajectoryControllerAction, TrajectoryControllerGoal

class SimplePlannerNode:
    def __init__(self):
        self._publish_rate = rospy.get_param('~publish_rate', 10)
        self.client = actionlib.SimpleActionClient('move_base_mod', TrajectoryControllerAction)
        rospy.Subscriber("goal", PoseStamped, self.goal_cb)

    def goal_cb(self, msg : PoseStamped):
        self.send_msg(msg)

    def send_msg(self, msg : PoseStamped):
        goal = TrajectoryControllerGoal()
        goal.poses += [msg]
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()

    def run(self):
        self.client.wait_for_server()
        rate = rospy.Rate(self._publish_rate)
        rospy.spin()
        # while not rospy.is_shutdown():
        #     rate.sleep()

def main():
    rospy.init_node('simple_planner_node')

    node = SimplePlannerNode()
    node.run()


if __name__ == '__main__':
    main()
