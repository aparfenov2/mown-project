import rospy
import json

import tf2_ros

import std_msgs.msg
from engix_msgs.msg import CoverageTask, DubinsPlanningTask
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32, PoseStamped

import utm
from typing import Tuple

"""
interface Backend 2 for Flutter UI
"""

def convert_to_utm(latitude: float, longitude: float) -> Tuple[float, float]:
    easting, northing, _, _ = utm.from_latlon(latitude, longitude)
    return (easting, northing)

class BackendNode:
    def __init__(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.map_frame = rospy.get_param('~map_frame', 'odom')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'base_link')

        self.pub_mown_task_move_to_point = rospy.Publisher(
            "/global_planner/dubins_planning_task",
            DubinsPlanningTask,
            queue_size=2,
        )
        self.pub_mown_task = rospy.Publisher(
            "/global_planner/coverage_planning_task",
            CoverageTask,
            queue_size=2,
        )

        self.ui_updates_pub = rospy.Publisher(
            "/ui_flutter_updates",
            std_msgs.msg.String,
            queue_size=2,
        )

        self.pathgen_props = {
            "step_size": 0.1,
            "angle": 30,
            "auto_angle": True
        }

        self.ui_state = {
            "op_mode_state" : "manual",
            "progress": 10,
            "curr_speed": 0.5,
            "pathgen_props": self.pathgen_props
        }

        rospy.Timer(rospy.Duration(0.1), self.uiUpdatesTimerCb)

        rospy.Subscriber("/ui_flutter_in", std_msgs.msg.String, self.cmdUiCb)


    def cmdUiCb(self, cmd_msg: std_msgs.msg.String):
        jss = cmd_msg.data
        js = json.loads(jss)
        rospy.loginfo(f"RECEIVED: {jss}")

        cmd = js["cmd"]

        if cmd == "switch_op_mode":
            self.ui_state["op_mode_state"] = "auto" if js["displayed_value"] == "manual" else "manual"

        if cmd == "build_path":
            self.sendUserAlert("build_path is not implemented. Will be shown after Start.")

        if cmd == "submit_task":
            self.on_submit_task(js)

        if cmd == "stop":
            self.sendUserAlert("stop is not implemented")

        if cmd == "set_desired_speed":
            self.ui_state["curr_speed"] = float(js["desired_speed"])

        if cmd == "set_pathgen_props":
            self.pathgen_props["step_size"] = float(js["step_size"])
            self.pathgen_props["angle"] = float(js["angle"])
            self.pathgen_props["auto_angle"] = bool(js["auto_angle"])

    def on_submit_task(self, js):
        task = js["task"]
        if "target_point" in task and task["target_point"] is not None:
            trg_x, trg_y = convert_to_utm(
                    task["target_point"]["coordinates"][1],
                    task["target_point"]["coordinates"][0]
                    )
            self.exec_go_to_point(trg_x, trg_y)
            return
        pts = []
        if len(task["polygons"]) == 0:
            self.sendUserAlert("len(polygons) == 0")
            return
        if len(task["polygons"]) > 1:
            self.sendUserAlert("len(polygons) > 1. Using only 1st poly.")
        class _A: pass
        for poly in task["polygons"]:
            for p in poly:
                pp = _A()
                pp.x, pp.y = convert_to_utm(p["coordinates"][1], p["coordinates"][0])
                pts.append(pp)
            break
        self.exec_coverage(pts)

    def sendUserAlert(self, msg):
        rospy.logerror("sending to user: %s", msg)
        js = {
            "error": msg
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(js)
        self.ui_updates_pub.publish(msg)

    def uiUpdatesTimerCb(self, arg0):
        js = {
            "state": self.ui_state
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(js)
        self.ui_updates_pub.publish(msg)

    def get_current_pos(self):
        trans = self.tfBuffer.lookup_transform(self.map_frame, self.base_link_frame, rospy.Time())
        return trans.transform.translation.x, trans.transform.translation.y

    def exec_coverage(self, points):
        assert len(points) > 2, str(len(points))
        message = CoverageTask()
        message.header.stamp = rospy.get_rostime()
        message.auto_angle = True
        message.approximate = True
        for p in points:
            new_point = Point32()
            new_point.x = p.x
            new_point.y = p.y
            message.target_polygon.points.append(new_point)
        self.pub_mown_task.publish(message)
        rospy.loginfo("emit execute msg %s", str(message))


    def exec_go_to_point(self, trg_x, trg_y):
        message = DubinsPlanningTask()
        message.header.stamp = rospy.get_rostime()
        # x, y = self.get_current_pos()
        message.target_pose.x = trg_x
        message.target_pose.y = trg_y
        message.target_speed  = self.ui_state["curr_speed"]
        message.turning_radius  = 1.0
        message.step_size  = 0.5
        self.pub_mown_task_move_to_point.publish(message)
        rospy.loginfo("Published DubinsPlanningTask msg %s", str(message))

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("backend_node2", anonymous=True)
    BackendNode().main()
