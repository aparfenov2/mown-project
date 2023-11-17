# временная заглушка для подсистемы планирования
# принимает RouteTaskPolygon, и запускает движение через pid контроллер
import rospy
import tf
import tf2_ros
from engix_msgs.msg import RouteTaskToPoint, RouteTaskPolygon
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PlanningStubNode:
    def __init__(self):
        rospy.init_node('planning_stub', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pub = rospy.Publisher("path_to_follow", Path, queue_size=10)
        rospy.Subscriber(rospy.get_param('/planner/topics/task_polygon_planning'), RouteTaskPolygon, self.routeTaskCb)
        debug = rospy.get_param("~debug", False)
        if debug:
            rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.pointCb)
        rospy.loginfo("planning_stub started")

    def get_current_pos(self):
        trans = self.tfBuffer.lookup_transform("odom", 'base_link', rospy.Time())
        return trans.transform.translation.x, trans.transform.translation.y

    def pointCb(self, point_msg):
        pt = point_msg.pose.position
        self.pub_path_to_pt(pt)

    def routeTaskCb(self, task:RouteTaskToPoint):
        pt = task.target_polygon.points[0]
        self.pub_path_to_pt(pt)

    def pub_path_to_pt(self, pt):

        rospy.loginfo(f"pub: {pt}")

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        x, y = self.get_current_pos()
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        msg.poses.append(pose)

        pose = PoseStamped()
        pose.pose.position.x = pt.x
        pose.pose.position.y = pt.y
        pose.pose.position.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        msg.poses.append(pose)

        self.pub.publish(msg)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    PlanningStubNode().main()

