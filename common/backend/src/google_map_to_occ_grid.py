
"""
сервис растеризует спутниковую карту Гугл, и публикует ее как сетку препятсвий (occupancy_grid) для отображения в foxglove.
Для растеризации использует Selenium и Firefox, работаюший в фоновом процессе.
input: GPS coordinates
output: occupancy_grid
"""

import math
import cv2
import numpy as np

import rospy
import tf_conversions
import tf2_ros

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import NavSatFix
from foxglove_msgs.msg import Grid, PackedElementField

from selenium.common.exceptions import JavascriptException
from selenium import webdriver as selenium_webdriver
from selenium.webdriver.firefox.options import Options as selenium_options
from selenium.webdriver.common.desired_capabilities import DesiredCapabilities as selenium_DesiredCapabilities

# https://gis.stackexchange.com/questions/7430/what-ratio-scales-do-google-maps-zoom-levels-correspond-to

class WebPageRenderer:
    def __init__(self, z=17) -> None:

        w, h = 512, 512
        self.resolution = 0.1
        z = self.calc_z(55)
        rospy.loginfo("set driver window size to %dx%d, z (calculated) = %f", w, h, z)

        self.cnt = 0
        # https:#towardsdatascience.com/google-maps-feature-extraction-with-selenium-faa2b97b29af
        browser_options = selenium_options()
        # browser_options.add_argument("--headless")
        capabilities_argument = selenium_DesiredCapabilities().FIREFOX
        capabilities_argument["marionette"] = True

        self.driver = selenium_webdriver.Firefox(
            options=browser_options,
            # firefox_binary="firefox/firefox",
            # capabilities=capabilities_argument
        )
        self.driver.set_window_size(w, h)

    def calc_z(self, lat):
        return np.log(156543.03392 * np.cos(lat * np.pi / 180) / self.resolution) / np.log(2) + 1

    def get_window_size(self):
        wnd_sz = self.driver.get_window_size()
        w = int(wnd_sz["width"])
        h = int(wnd_sz["height"])
        return w, h

    def load_map_page(self, lat=43.640722, lng=-79.3811892):
        z = self.calc_z(lat)

        url = f"https:/www.google.com/maps/@{lat:0.7f},{lng:0.7f},{z:0.4f}z"
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
        # self.driver.save_screenshot(f"/cdir/google_map_{self.cnt}.png")
        self.cnt += 1
        png = self.driver.get_screenshot_as_png()
        png = np.frombuffer(png, dtype='uint8')
        img = cv2.imdecode(png, cv2.IMREAD_UNCHANGED)
        rospy.loginfo(f"got {img.shape} {img.dtype} image")
        return img

    def render_map_with_coords(self, lat=43.640722, lng=-79.3811892):
        self.load_map_page(lat, lng)
        return self.capture_page()

class Node:

    def __init__(self):

        self.map_frame_id = rospy.get_param("~map_frame_id", "map")
        self.world_frame_id = rospy.get_param("~world_frame_id", "world")
        self.robot_frame_id = rospy.get_param("~robot_frame_id", "base_link")
        timer_period = float(rospy.get_param("~timer_period", 5.0))
        self.sensitivity = float(rospy.get_param("~sensitivity", 1e-6))
        z = float(rospy.get_param("~z", 18))

        self.renderer = WebPageRenderer(z=z)

        self.last_lat, self.last_lon = None, None
        self.prev_last_lat, self.prev_last_lon = None, None
        self.resolution = 0.1

        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self._map_pub = rospy.Publisher('map', Grid, queue_size=1, latch=True)
        self._occ_map_pub = rospy.Publisher('occ_map', OccupancyGrid, queue_size=1, latch=True)

        rospy.Subscriber('fix', NavSatFix, self.gpsCallback)
        rospy.Timer(rospy.Duration(timer_period), self.timerCallback)
        rospy.loginfo('ready')

    def to_message(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
        img = cv2.flip(img, 0)

        width = img.shape[1]
        height = img.shape[0]
        origin_x = -width/2 * self.resolution
        origin_y = -height/2 * self.resolution

        grid_msg = Grid()
        # https:#github.com/foxglove/schemas/blob/main/schemas/ros1/Grid.msg

        # Set up the header.
        grid_msg.timestamp = rospy.Time.now()
        grid_msg.frame_id = self.map_frame_id

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.pose = Pose(Point(origin_x, origin_y, 0),
                               Quaternion(0, 0, 0, 1))
        grid_msg.column_count = width
        grid_msg.cell_size.x = self.resolution
        grid_msg.cell_size.y = self.resolution
        grid_msg.row_stride  = width * img.shape[-1] # RGBA image
        grid_msg.cell_stride = img.shape[-1]

        # https:#github.com/foxglove/tutorials/blob/main/studio/rgba-point-cloud/example.py
        grid_msg.fields = [
            PackedElementField("red",   0, PackedElementField.UINT8),
            PackedElementField("green", 1, PackedElementField.UINT8),
            PackedElementField("blue",  2, PackedElementField.UINT8),
            PackedElementField("alpha", 3, PackedElementField.UINT8),
        ]

        grid_msg.data = list(img.flatten())
        return grid_msg

    def to_occ_message(self, img):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.flip(img, 0)

        width = img.shape[1]
        height = img.shape[0]

        origin_x = -width/2 * self.resolution
        origin_y = -height/2 * self.resolution

        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = self.map_frame_id

        # .info is a nav_msgs/MapMetaData message.
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = width
        grid_msg.info.height = height

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.info.origin = Pose(Point(origin_x, origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        img = img.astype(np.float32)
        img /= 255.
        img *= 100.
        img = img.astype(np.uint8)
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
            last_trans = self.tfBuffer.lookup_transform(self.world_frame_id, self.robot_frame_id, timestamp, rospy.Duration(3.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logerr(f"failed to get transform from {self.world_frame_id} to {self.robot_frame_id} for time {timestamp}: reason {ex}")
            return

        img = self.renderer.render_map_with_coords(self.last_lat, self.last_lon)

        grid_msg = self.to_message(img)
        self._map_pub.publish(grid_msg)

        grid_msg = self.to_occ_message(img)
        self._occ_map_pub.publish(grid_msg)

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

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('google_map_cap_node', anonymous=True)
    Node().run()
