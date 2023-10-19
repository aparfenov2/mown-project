
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
import utm
from dynamic_reconfigure.server import Server
from backend.cfg import GoogleMapConfig

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
    def __init__(self,
        z=19.95,
        m=100,
        headless=True,
        w=512, h=512,
        resolution=0.1,
        satellite_mode=False,
        load_cap_delay=3.0
        ) -> None:

        self.resolution = resolution
        self.z = z
        self.m = m
        self.load_cap_delay = load_cap_delay
        self.satellite_mode = satellite_mode
        _z = self.calc_z(55)
        rospy.loginfo("set driver window size to %dx%d, z (calculated) = %f, self.z = %f", w, h, _z, self.z)

        self.cnt = 0
        # https:#towardsdatascience.com/google-maps-feature-extraction-with-selenium-faa2b97b29af
        browser_options = selenium_options()
        if headless:
            browser_options.add_argument("--headless")
        capabilities_argument = selenium_DesiredCapabilities().FIREFOX
        capabilities_argument["marionette"] = True

        self.driver = selenium_webdriver.Firefox(
            options=browser_options,
            # firefox_binary="firefox/firefox",
            # capabilities=capabilities_argument
        )
        self.driver.set_window_size(w, h)

    def calc_z(self, lat):
        return np.log(156543.03392 * np.cos(lat * np.pi / 180) / self.resolution) / np.log(2)

    def get_window_size(self):
        wnd_sz = self.driver.get_window_size()
        w = int(wnd_sz["width"])
        h = int(wnd_sz["height"])
        return w, h

    def load_map_page(self, lat=43.640722, lon=-79.3811892):
        # z = self.calc_z(lat)
        if not self.satellite_mode:
            url = f"https:/www.google.com/maps/@{lat:0.7f},{lon:0.7f},{self.z:0.4f}z"
        else:
            url = f"https://www.google.com/maps/@{lat:0.7f},{lon:0.7f},{self.m:0.0f}m/data=!3m1!1e3?entry=ttu"
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
        # rospy.loginfo(f"got {img.shape} {img.dtype} image")
        return img

    def render_map_with_coords(self, lat, lon):
        self.load_map_page(lat, lon)
        rospy.sleep(self.load_cap_delay)
        return self.capture_page()







class Node:

    def __init__(self):

        self.map_frame_id = rospy.get_param("~map_frame_id", "map")
        self.world_frame_id = rospy.get_param("~world_frame_id", "world")
        self.timer_period = float(rospy.get_param("~timer_period", 5.0))
        self.sensitivity = float(rospy.get_param("~sensitivity", 1e-6))
        self.orig_lat_override = rospy.get_param("~orig_lat_override", None)
        if self.orig_lat_override is not None:
            self.orig_lat_override = float(self.orig_lat_override)
        self.orig_lon_override = rospy.get_param("~orig_lon_override", None)
        if self.orig_lon_override is not None:
            self.orig_lon_override = float(self.orig_lon_override)
        self.lat_offset = float(rospy.get_param("~lat_offset", 0))
        self.lon_offset = float(rospy.get_param("~lon_offset", 0))

        run_cfg_srv = bool(rospy.get_param("~run_cfg_srv", False))
        satellite_mode = bool(rospy.get_param("~satellite_mode", False))
        headless = bool(rospy.get_param("~headless", False))
        wnd_w, wnd_h = rospy.get_param("~window_size", "512x512").split('x')
        load_cap_delay = float(rospy.get_param("~load_cap_delay", 3.0))
        resolution = float(rospy.get_param("~resolution", 0.1))
        wnd_w, wnd_h = float(wnd_w), float(wnd_h)
        z = float(rospy.get_param("~z", 18))
        m = float(rospy.get_param("~m", 100))

        self.driver = WebPageRenderer(
            z=z, m=m,
            headless=headless,
            w=wnd_w,
            h=wnd_h,
            resolution=resolution,
            satellite_mode=satellite_mode,
            load_cap_delay=load_cap_delay
            )

        self.last_lat, self.last_lon = None, None
        self.prev_last_lat, self.prev_last_lon = None, None
        self.resolution = 0.1
        self.origin_utm_x, self.origin_utm_y = None, None

        self.br = tf2_ros.TransformBroadcaster()

        self._map_pub = rospy.Publisher('map', Grid, queue_size=1, latch=True)
        self._occ_map_pub = rospy.Publisher('occ_map', OccupancyGrid, queue_size=1, latch=True)

        if run_cfg_srv:
            self.srv = Server(GoogleMapConfig, self.cfgCallback)

        rospy.Subscriber('fix', NavSatFix, self.gpsCallback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)
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

    def cfgCallback(self, config, _):
        rospy.loginfo("Reconfigure Request: z=%f, m=%f", config['z'], config['m'])
        self.driver.z = config['z']
        self.driver.m = config['m']
        return config

    def timerCallback(self, _):
        # if self.last_lat is None or (
        #     self.prev_last_lat is not None and
        #     math.isclose(self.last_lat, self.prev_last_lat, rel_tol=0, abs_tol=self.sensitivity) and
        #     math.isclose(self.last_lon, self.prev_last_lon, rel_tol=0, abs_tol=self.sensitivity)
        #     ):
        #     return
        # self.prev_last_lat, self.prev_last_lon = self.last_lat, self.last_lon
        if self.origin_utm_x is None or self.last_lat is None:
            return
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
            rospy.Timer(rospy.Duration(self.timer_period), self.timerCallback)

        last_utm_x, last_utm_y, _,_ = utm.from_latlon(self.last_lat, self.last_lon)

        img = self.driver.render_map_with_coords(self.last_lat, self.last_lon)

        grid_msg = self.to_message(img)
        self._map_pub.publish(grid_msg)

        grid_msg = self.to_occ_message(img)
        self._occ_map_pub.publish(grid_msg)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame_id
        t.child_frame_id = self.map_frame_id
        t.transform.translation.x = (last_utm_x - self.origin_utm_x)
        t.transform.translation.y = (last_utm_y - self.origin_utm_y)
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

    def gpsCallback(self, msg):
        if self.lat_offset == 0 and self.orig_lat_override is not None:
            self.lat_offset = self.orig_lat_override - msg.latitude
        if self.lon_offset == 0 and self.orig_lon_override is not None:
            self.lon_offset = self.orig_lon_override - msg.longitude
        self.last_lat, self.last_lon = msg.latitude + self.lat_offset, msg.longitude + self.lon_offset
        self.last_timestamp = msg.header.stamp
        if self.origin_utm_x is None:
            self.origin_utm_x, self.origin_utm_y, _,_ = utm.from_latlon(self.last_lat, self.last_lon)

        # last_utm_x, last_utm_y, _,_ = utm.from_latlon(self.last_lat, self.last_lon)
        # rospy.loginfo("x=%f, y=%f", last_utm_x, last_utm_y)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('google_map_cap_node', anonymous=True)
    Node().run()
