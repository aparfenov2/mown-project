
"""
сервис растеризует спутниковую карту Гугл, и публикует ее как сетку препятсвий (occupancy_grid) для отображения в foxglove.
Для растеризации использует Selenium и Firefox, работаюший в фоновом процессе.
input: GPS coordinates
output: occupancy_grid
"""

import re
import math
import cv2
import numpy as np
import utm
import numpy as np

import rospy
import tf_conversions
import tf2_ros

from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import NavSatFix
from foxglove_msgs.msg import Grid, PackedElementField

from selenium.common.exceptions import JavascriptException
from selenium import webdriver as selenium_webdriver
from selenium.webdriver.firefox.options import Options as selenium_options
from selenium.webdriver.common.desired_capabilities import DesiredCapabilities as selenium_DesiredCapabilities
from selenium.webdriver.common.action_chains import ActionChains
from selenium.webdriver.common.actions.action_builder import ActionBuilder
from selenium.webdriver.common.by import By
from selenium.webdriver.common.actions.mouse_button import MouseButton


# https://stackoverflow.com/questions/1585525/how-to-find-the-intersection-point-between-a-line-and-a-rectangle
# /**
#  * Finds the intersection point between
#  *     * a rectangle centered in point B
#  *       with sides parallel to the x and y axes
#  *     * a line passing through points A and B (the center of the rectangle)
#  *
#  * @param width: rectangle width
#  * @param height: rectangle height
#  * @param xB; rectangle center x coordinate
#  * @param yB; rectangle center y coordinate
#  * @param xA; point A x coordinate
#  * @param yA; point A y coordinate
#  * @author Federico Destefanis
#  * @see <a href="https://stackoverflow.com/a/31254199/2668213">based on</a>
#  */

def lineIntersectionOnRect(width, height, xB, yB, xA, yA):

    w = width / 2
    h = height / 2

    dx = xA - xB
    dy = yA - yB

    # if A=B return B itself
    if (dx == 0 and dy == 0):
        return xB, yB

    tan_phi = h / w
    tan_theta = abs(dy / dx)

    # tell me in which quadrant the A point is
    qx = np.sign(dx)
    qy = np.sign(dy)

    if (tan_theta > tan_phi):
        xI = xB + (h / tan_theta) * qx
        yI = yB + h * qy
    else:
        xI = xB + w * qx
        yI = yB + w * tan_theta * qy

    return xI, yI

class WebPageRenderer:
    def __init__(self, calib_lat, calib_lng, z=17) -> None:
        # https://towardsdatascience.com/google-maps-feature-extraction-with-selenium-faa2b97b29af
        browser_options = selenium_options()
        browser_options.add_argument("--headless")
        capabilities_argument = selenium_DesiredCapabilities().FIREFOX
        capabilities_argument["marionette"] = True

        self.driver = selenium_webdriver.Firefox(
            options=browser_options,
            # firefox_binary="firefox/firefox",
            # capabilities=capabilities_argument
        )
        self.cnt = 0
        self.scale_x = 0
        self.scale_y = 0
        self.z = z

        self.driver.set_window_size(1920, 1080)
        w, h = self.get_window_size()
        rospy.loginfo("set driver window size to %dx%d", w, h)

        self.load_map_page(calib_lat, calib_lng)
        rospy.sleep(0.4)
        self.calibrate_scales()


    def load_map_page(self, lat=43.640722, lng=-79.3811892):
        url = 'https://www.google.com/maps/@' + \
            str(lat) + ',' + str(lng) + ',' + str(self.z) + 'z'
        rospy.loginfo("loading url = %s", url)
        self.driver.get(url)

    def exec_js(self, js_string):
        try:
            self.driver.execute_script(js_string)
        except JavascriptException as ex:
            rospy.logdebug("javascript err: js = %s, err = %s", js_string, ex)

    def remove_ui(self):
        # # Remove omnibox
        self.exec_js(
            "element = document.getElementById(\"omnibox-container\"); element.remove();")
        # switch to satellite button
        self.exec_js(
            "element = document.getElementsByClassName(\"F63Kk\"); element[0].remove();")
        # zoom btns
        self.exec_js(
            "element = document.getElementsByClassName(\"app-vertical-widget-holder\"); element[0].remove();")
        self.exec_js(
            "element = document.getElementById(\"runway-expand-button\"); element.remove();")
        # footer
        self.exec_js(
            "element = document.getElementsByClassName(\"scene-footer-container\"); element[0].remove();")

    def capture_page(self):
        self.remove_ui()
        self.driver.save_screenshot(f"/cdir/google_map_{self.cnt}.png")
        self.cnt += 1
        png = self.driver.get_screenshot_as_png()
        png = np.frombuffer(png, dtype='uint8')
        img = cv2.imdecode(png, cv2.IMREAD_UNCHANGED)
        rospy.loginfo(f"got {img.shape} {img.dtype} image")
        return img

    def get_cur_lat_lng(self):
        m = re.search(r"@(\d+\.\d+)\,(\d+\.\d+)", self.driver.current_url)
        if not m:
            return None, None
        cur_lat = float(m.group(1))
        cur_lng = float(m.group(2))
        return cur_lat, cur_lng

    def wait_url_changed(self):
        current_url = self.driver.current_url
        cnt = 0
        last_lat, last_lng = self.get_cur_lat_lng()
        abs_tol = 0.0001
        while True:
            if cnt >= 100:
                break
            if current_url != self.driver.current_url:
                cur_lat, cur_lng = self.get_cur_lat_lng()
                if math.isclose(cur_lat, last_lat, abs_tol=abs_tol) and \
                        math.isclose(cur_lng, last_lng, abs_tol=abs_tol):
                    rospy.logwarn("url changed, but lat/lng is the same, was url = %s, now = %s",
                                  current_url, self.driver.current_url)
                    current_url = self.driver.current_url
                else:
                    break
            rospy.sleep(0.1)
            cnt += 1
            if cnt % 20 == 0:
                rospy.loginfo("still waiting for url change, steps = %d", cnt)
        if cnt >= 100:
            rospy.logerr(
                "lat/lng wasnt changed after %d steps, url = %s", cnt, current_url)
            return None
        return current_url

    def get_window_size(self):
        wnd_sz = self.driver.get_window_size()
        w = int(wnd_sz["width"])
        h = int(wnd_sz["height"])
        return w, h

    def _do_move(self, start_x, start_y, end_x, end_y):
        rospy.loginfo("moving start_x = %d, start_y = %d, end_x = %d, end_y = %d, curr_url = %s",
                      start_x, start_y, end_x, end_y, self.driver.current_url)
        action = ActionBuilder(self.driver)
        action.pointer_action.move_to_location(start_x, start_y)
        action.pointer_action.pointer_down(MouseButton.LEFT)
        action.pointer_action.move_to_location(end_x, end_y)
        action.pointer_action.pointer_up(MouseButton.LEFT)
        action.perform()

        if not self.wait_url_changed():
            return False
        rospy.loginfo("moved: curr_url = %s", self.driver.current_url)
        return True

    def get_bb(self):
        w, h = self.get_window_size()
        bb_x0, bb_y0 = 100, 100
        bb_x1, bb_y1 = w-100, h-100 - 85
        return bb_x0, bb_y0, bb_x1, bb_y1

    def do_move(self, start_x, start_y, end_x, end_y):
        bb_x0, bb_y0, bb_x1, bb_y1 = self.get_bb()
        if (bb_x0 <= start_x <= bb_x1 and
            bb_y0 <= start_y <= bb_y1 and
            bb_x0 <= end_x <= bb_x1 and
            bb_y0 <= end_y <= bb_y1):
            return self._do_move(start_x, start_y, end_x, end_y)

        bb_xc, bb_yc = (bb_x0 + bb_x1) // 2, (bb_y0 + bb_y1) // 2
        assert start_x == bb_xc and start_y == bb_yc

        part = 0
        while not (bb_x0 <= end_x <= bb_x1 and
            bb_y0 <= end_y <= bb_y1):

            xi, yi = lineIntersectionOnRect(bb_x1 - bb_x0, bb_y1 - bb_y0, start_x, start_y, end_x, end_y)
            xi, yi = int(xi), int(yi)
            dxi, dyi = xi - start_x, yi - start_y
            if not self._do_move(start_x, start_y, xi, yi):
                rospy.logerr("failed to move part %d: start_x = %d, start_y = %d, xi = %d, yi = %d", part, start_x, start_y, xi, yi)
                return
            end_x -= dxi
            end_y -= dyi
            part += 1

        if not self._do_move(start_x, start_y, end_x, end_y):
            rospy.logerr("failed to move part %d: start_x = %d, start_y = %d, xi = %d, yi = %d", part, start_x, start_y, end_x, end_y)
            return False
        return True


    def calibrate_scales(self):
        """determines lat, lng deltas for fixed dx, dy"""
        # driver should be initialzed, initial page loaded

        rospy.loginfo("stared calibration, curr_url = %s",
                      self.driver.current_url)

        w, h = self.get_window_size()
        cur_lat, cur_lng = self.get_cur_lat_lng()
        bb_x0, bb_y0, bb_x1, bb_y1 = self.get_bb()
        start_x, start_y = bb_x0, bb_y0
        end_x, end_y = bb_x1, bb_y1
        dx, dy = end_x - start_x, end_y - start_y
        rospy.loginfo("dx = %d, dy = %d, start_x = %d, start_y = %d, end_x = %d, end_y = %d, w = %d, h = %d",
                      dx, dy, start_x, start_y, end_x, end_y, w, h)
        assert dx > 0 and dy > 0

        _dx, _dy = dx, dy
        for move_step in range(3):
            rospy.loginfo("move step %d", move_step)
            if not self._do_move(start_x, start_y, end_x, end_y):
                return
            dx, dy = dx + _dx, dy + _dy
        new_lat, new_lng = self.get_cur_lat_lng()
        self.scale_x = (new_lng - cur_lng) / dx
        self.scale_y = (new_lat - cur_lat) / dy

        rospy.loginfo("new_lat = %f, new_lng = %f, cur_lat = %f, cur_lng = %f",
                      new_lat, new_lng, cur_lat, cur_lng)

        rospy.loginfo("calibrated scales for z = %d: self.scale_x = %f, self.scale_y = %f",
                      self.z, self.scale_x, self.scale_y)

        self.move_to_position(cur_lat, cur_lng)
        self.load_map_page(cur_lat, cur_lng)
        rospy.loginfo("position restored to %s", self.driver.current_url)


    def move_to_position(self, lat, lng):
        if self.scale_x == 0:
            rospy.logerr("need to calibrate scales first")
            return

        min_dx, min_dy = 4, 4
        cur_lat, cur_lng = self.get_cur_lat_lng()
        dy = int((lat - cur_lat) / self.scale_y)
        dx = int((lng - cur_lng) / self.scale_x)

        if abs(dy) < min_dy and abs(dx) < min_dx:
            rospy.logerr("diff too small: dx = %d, dy = %d, cur_lat = %f, cur_lng = %f, lat = %f, lng = %f",
                         dx, dy, cur_lat, cur_lng, lat, lng)
            return

        rospy.loginfo("requested lat = %f, lng = %f, cur_lat = %f, cur_lng = %f, dx = %d, dy = %d ",
                      lat, lng, cur_lat, cur_lng, dx, dy)

        bb_x0, bb_y0, bb_x1, bb_y1 = self.get_bb()
        bb_xc, bb_yc = (bb_x0 + bb_x1) // 2, (bb_y0 + bb_y1) // 2
        if not self.do_move(bb_xc, bb_yc, bb_xc + dx, bb_yc + dy):
            return

        rospy.loginfo("url after: %s", self.driver.current_url)

        # eval error
        cur_lat, cur_lng = self.get_cur_lat_lng()
        cur_x, cur_y, _, _ = utm.from_latlon(cur_lat, cur_lng)
        _x, _y, _, _ = utm.from_latlon(lat, lng)
        rospy.loginfo("abs error: d_lat = %f, d_lng = %f, dx, m = %f, dy, m = %d, cur_lat = %f, cur_lng = %f, lat = %f, lng = %f",
                      lat - cur_lat, lng - cur_lng, _x - cur_x, _y - cur_y, cur_lat, cur_lng, lat, lng)

    def render_map_with_coords(self, lat=43.640722, lng=-79.3811892):
        self.move_to_position(lat, lng)
        return self.capture_page()


class Node:

    def __init__(self):

        self.map_frame_id = rospy.get_param("~map_frame_id", "map")
        self.world_frame_id = rospy.get_param("~world_frame_id", "world")
        self.robot_frame_id = rospy.get_param("~robot_frame_id", "base_link")
        calib_lat = float(rospy.get_param("~calib_lat", 55.164440))
        calib_lng = float(rospy.get_param("~calib_lng", 61.436844))
        timer_period = float(rospy.get_param("~timer_period", 5.0))
        self.sensitivity = float(rospy.get_param("~sensitivity", 1e-6))
        z = float(rospy.get_param("~z", 18))
        self.last_lat, self.last_lon = None, None
        self.prev_last_lat, self.prev_last_lon = None, None

        self.renderer = WebPageRenderer(z=z, calib_lat=calib_lat, calib_lng=calib_lng)

        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self._map_pub = rospy.Publisher('map', Grid, queue_size=1, latch=True)
        # self._map_data_pub = rospy.Publisher('map_metadata',
        #                                      MapMetaData, latch=True)
        rospy.Subscriber('fix', NavSatFix, self.gpsCallback)
        # rospy.Timer(rospy.Duration(timer_period), self.timerCallback)
        rospy.loginfo('ready')

    def to_message(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
        img = cv2.flip(img, 0)

        width = img.shape[1]
        height = img.shape[0]
        resolution = 0.1
        origin_x = -width/2 * resolution
        origin_y = -height/2 * resolution

        grid_msg = Grid()
        # https://github.com/foxglove/schemas/blob/main/schemas/ros1/Grid.msg

        # Set up the header.
        grid_msg.timestamp = rospy.Time.now()
        grid_msg.frame_id = self.map_frame_id

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.pose = Pose(Point(origin_x, origin_y, 0),
                             Quaternion(0, 0, 0, 1))
        grid_msg.column_count = width
        grid_msg.cell_size.x = resolution
        grid_msg.cell_size.y = resolution
        grid_msg.row_stride = width * img.shape[-1]  # RGBA image
        grid_msg.cell_stride = img.shape[-1]

        # https://github.com/foxglove/tutorials/blob/main/studio/rgba-point-cloud/example.py
        grid_msg.fields = [
            PackedElementField("red",   0, PackedElementField.UINT8),
            PackedElementField("green", 1, PackedElementField.UINT8),
            PackedElementField("blue",  2, PackedElementField.UINT8),
            PackedElementField("alpha", 3, PackedElementField.UINT8),
        ]

        grid_msg.data = list(img.flatten())
        return grid_msg

    def timerCallback(self, _):
        # if self.last_lat is None or (
        #     self.prev_last_lat is not None and
        #     math.isclose(self.last_lat, self.prev_last_lat, rel_tol=0, abs_tol=self.sensitivity) and
        #     math.isclose(self.last_lon, self.prev_last_lon, rel_tol=0, abs_tol=self.sensitivity)
        #     ):
        #     return
        # self.prev_last_lat, self.prev_last_lon = self.last_lat, self.last_lon

        try:
            timestamp = self.last_timestamp
            last_trans = self.tfBuffer.lookup_transform(
                self.world_frame_id, self.robot_frame_id, timestamp, rospy.Duration(3.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logerr(
                f"failed to get transform from {self.world_frame_id} to {self.robot_frame_id} for time {timestamp}: reason {ex}")
            return

        img = self.renderer.render_map_with_coords(
            self.last_lat, self.last_lon, self.z)

        grid_msg = self.to_message(img)
        self._map_pub.publish(grid_msg)

        t = last_trans
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame_id
        t.child_frame_id = self.map_frame_id
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

    def gpsCallback(self, msg):
        self.last_lat, self.last_lon = msg.latitude, msg.longitude
        self.last_timestamp = msg.header.stamp
        self.renderer.render_map_with_coords(self.last_lat, self.last_lon)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('google_map_cap_node', anonymous=True)
    Node().run()
