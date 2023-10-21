# Контроллер джвижения по пути покрытия.
# Управляющая логика для работы в паре с move_base_flex.

# https://uos.github.io/mbf_docs/tutorials/beginner/beyond_relay/
# https://uos.github.io/mbf_docs/tutorials/beginner/path_planning/

import actionlib
import rospy
import mbf_msgs.msg as mbf_msgs
import mbf_msgs.srv as srv_msgs
import geometry_msgs.msg as geometry_msgs
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from visualization_msgs.msg import Marker

class Main:
    def __init__(self, spiral_get_ac, dwa_get_ac, dwa_exe_ac) -> None:
        self.spiral_get_ac = spiral_get_ac
        self.dwa_get_ac = dwa_get_ac
        self.dwa_exe_ac = dwa_exe_ac
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.marker_pub = rospy.Publisher("/current_goal_marker", Marker, queue_size = 2)
        rospy.wait_for_service('DWA_node/check_path_cost')
        self.check_path_svc = rospy.ServiceProxy('DWA_node/check_path_cost', srv_msgs.CheckPath)

    def check_path(self, path):
        # req = mbf_msgs.CheckPath.Request
        # req.path = path
        # req.costmap = 2 # global
        rsp = self.check_path_svc(path=path, costmap=2)
        return rsp.cost < 2 # LETHAL

    def publish_current_goal(self, goal_x, goal_y):
        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = 0

        # Set the scale of the marker
        scale = 0.3
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.marker_pub.publish(marker)


    @staticmethod
    def create_pose(x, y, z, xx, yy, zz, ww, frame="odom"):
        pose = geometry_msgs.PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = xx
        pose.pose.orientation.y = yy
        pose.pose.orientation.z = zz
        pose.pose.orientation.w = ww
        return pose

    @staticmethod
    def create_path_goal(path, tolerance_from_action, dist_tolerance, angle_tolerance):
        goal = mbf_msgs.ExePathGoal()
        goal.path = path
        goal.tolerance_from_action = tolerance_from_action
        goal.dist_tolerance = dist_tolerance
        goal.angle_tolerance = angle_tolerance
        return goal


    def exe_path(self, path):
        path_goal = self.create_path_goal(path, True, 0.5, 3.14/18.0)
        self.dwa_exe_ac.send_goal(path_goal)
        self.dwa_exe_ac.wait_for_result()
        return self.dwa_exe_ac.get_result()

    def get_robot_pose(self):
        target_frame = "odom"
        pose_stamped = self.create_pose(0, 0, 0, 0, 0, 0, 1, "base_link")
        transform = self.tf_buffer.lookup_transform(target_frame,
                                            # source frame:
                                            pose_stamped.header.frame_id,
                                            # get the tf at the time the pose was valid
                                            pose_stamped.header.stamp,
                                            # wait for at most 1 second for transform, otherwise throw
                                            rospy.Duration(1.0))

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        return pose_transformed

    def get_short_plan(self, pose):
        path_goal = mbf_msgs.GetPathGoal(target_pose=pose, tolerance=0.5, planner="NavfnROS")
        self.dwa_get_ac.send_goal(path_goal)
        self.dwa_get_ac.wait_for_result()
        return self.dwa_get_ac.get_result()

    def get_spiral_plan(self):
        pose = self.create_pose(-1.75, 0.74, 0, 0, 0, 0.539, 0.843)
        path_goal = mbf_msgs.GetPathGoal(target_pose=pose, tolerance=0.5, planner="SpiralSTC")
        self.spiral_get_ac.send_goal(path_goal)
        self.spiral_get_ac.wait_for_result()
        spiral_result = self.spiral_get_ac.get_result()
        if spiral_result is None or (spiral_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS):
            rospy.logerr("Unable to get spiral plan plan: %s", spiral_result.message if spiral_result is not None else "None")
            return None
        return spiral_result

    @staticmethod
    def euclidean_distance(x1,y1, x2,y2):
        return np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))

    def loop(self):
        # 1. call SpiralSTC planner, get coverage path
        spiral_result = self.get_spiral_plan()
        if spiral_result is None:
            return
        # print(spiral_result)
        spiral_path = spiral_result.path.poses
        current_goal_index = 0
        robot_radius = 0.3

        while current_goal_index < len(spiral_path):

            current_goal = spiral_path[current_goal_index]
            goal_x = current_goal.pose.position.x
            goal_y = current_goal.pose.position.y
            rospy.loginfo(f"getting plan for segment {current_goal_index} to (x={goal_x:3.2f} y={goal_y:3.2f})")
            self.publish_current_goal(goal_x, goal_y)
            # rospy.loginfo(f"current goal {current_goal}")

            current_pose = self.get_robot_pose()
            cx = current_pose.pose.position.x
            cy = current_pose.pose.position.y
            rospy.loginfo(f"current robot pose is (x={cx:3.2f} y={cy:3.2f})")
            dist = self.euclidean_distance(cx, cy, goal_x, goal_y)
            if dist < robot_radius:
                rospy.logwarn(f"goal pose (x={goal_x:3.2f} y={goal_y:3.2f}) is too close to the robot (x={cx:3.2f} y={cy:3.2f}). Go to next goal.")
                current_goal_index += 1
                continue

            short_result = self.get_short_plan(current_goal)
            if short_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS or \
                len(short_result.path.poses) < 1 :
                rospy.logerr(f"segment {current_goal_index}: Unable to get short path plan: {short_result.message}")

                # replanning
                spiral_result = self.get_spiral_plan()
                if spiral_result is None:
                    return
                spiral_path = spiral_result.path.poses
                current_goal_index = 0
                continue

            rospy.loginfo(f"got plan for segment {current_goal_index}: {len(short_result.path.poses)} pts. Start excuting.")
            # path_valid = self.check_path(short_result.path)
            # rospy.loginfo(f"path_valid {path_valid}")

            exe_path_result = self.exe_path(short_result.path)
            if exe_path_result is None or (exe_path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS):
                rospy.logerr(f"segment {current_goal_index}: Unable to complete exe: {exe_path_result.message if exe_path_result is not None else 'None'}")

                # replanning
                spiral_result = self.get_spiral_plan()
                if spiral_result is None:
                    return
                spiral_path = spiral_result.path.poses
                current_goal_index = 0
                continue
            # rospy.loginfo(f"finished executing segment {current_goal_index}")

            current_goal_index += 1

        rospy.loginfo("Spiral finished. Terminating.")


def main():
    rospy.init_node("spiral_followe_fsm")

    # move_base_flex exe path client
    dwa_exe_ac = actionlib.SimpleActionClient("DWA_node/exe_path", mbf_msgs.ExePathAction)
    dwa_exe_ac.wait_for_server(rospy.Duration(10))
    rospy.loginfo("Connected to DWA_node/exe_path")

    dwa_get_ac = actionlib.SimpleActionClient("DWA_node/get_path", mbf_msgs.GetPathAction)
    dwa_get_ac.wait_for_server(rospy.Duration(10))
    rospy.loginfo("Connected to DWA_node/get_path")

    # move base flex get path client
    spiral_get_ac = actionlib.SimpleActionClient("SpiralSTC_node/get_path", mbf_msgs.GetPathAction)
    spiral_get_ac.wait_for_server(rospy.Duration(10))
    rospy.loginfo("Connected to SpiralSTC_node/get_path")

    Main(spiral_get_ac, dwa_get_ac, dwa_exe_ac).loop()

    rospy.on_shutdown(lambda: dwa_exe_ac.cancel_all_goals())

if __name__ == '__main__':
    main()
