
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

from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import NavSatFix
from foxglove_msgs.msg import Grid, PackedElementField

from selenium import webdriver as selenium_webdriver
from selenium.webdriver.firefox.options import Options as selenium_options
from selenium.webdriver.common.desired_capabilities import DesiredCapabilities as selenium_DesiredCapabilities

class Node:

    def __init__(self):

        # https://towardsdatascience.com/google-maps-feature-extraction-with-selenium-faa2b97b29af
        browser_options = selenium_options()
        browser_options.add_argument("--headless")
        capabilities_argument = selenium_DesiredCapabilities().FIREFOX
        capabilities_argument["marionette"] = True

        self.browser = selenium_webdriver.Firefox(
            options=browser_options,
            # firefox_binary="firefox/firefox",
            # capabilities=capabilities_argument
        )

        self.map_frame_id = rospy.get_param("~map_frame_id", "map")
        self.world_frame_id = rospy.get_param("~world_frame_id", "world")
        origin_lat = float(rospy.get_param("~origin_lat"))
        origin_lon = float(rospy.get_param("~origin_lon"))
        timer_period = float(rospy.get_param("~timer_period", 5.0))
        self.sensitivity = float(rospy.get_param("~sensitivity", 1e-6))
        self.origin_utm_x, self.origin_utm_y, _,_ = utm.from_latlon(origin_lat, origin_lon)
        self.last_lat, self.last_lon = None, None
        self.prev_last_lat, self.prev_last_lon = None, None
        self.br = tf2_ros.TransformBroadcaster()

        self._map_pub = rospy.Publisher('map', Grid, queue_size=1, latch=True)
        # self._map_data_pub = rospy.Publisher('map_metadata',
        #                                      MapMetaData, latch=True)
        rospy.Subscriber('fix', NavSatFix, self.gpsCallback)
        rospy.Timer(rospy.Duration(timer_period), self.timerCallback)
        rospy.loginfo('ready')

    def call_firefox(self, lat = 43.640722, lng = -79.3811892, z = 17):

        # Build the URL
        url = 'https://www.google.com/maps/@' + str(lat) + ',' + str(lng) + ',' + str(z) + 'z'
        # Setting up the browser window size
        browser = self.browser
        browser.set_window_size(512, 512)
        rospy.loginfo(url)
        browser.get(url)

        # # Remove omnibox
        js_string = "var element = document.getElementById(\"omnibox-container\"); element.remove();"
        browser.execute_script(js_string)
        # # Remove username and icons
        # js_string = "var element = document.getElementById(\"vasquette\"); element.remove();"
        # browser.execute_script(js_string)
        # # Remove bottom scaling bar
        # js_string = "var element = document.getElementsByClassName(\"app viewcard-strip\"); element[0].remove();"
        # browser.execute_script(js_string)
        # # Remove attributions at the bottom
        # js_string = "var element = document.getElementsByClassName(\"scene footer-container\"); element[0].remove();"
        # browser.execute_script(js_string)

        browser.save_screenshot("/cdir/google_map.png")
        png = browser.get_screenshot_as_png()
        png = np.frombuffer(png, dtype='uint8')
        img = cv2.imdecode(png, cv2.IMREAD_UNCHANGED)
        rospy.loginfo(f"got {img.shape} {img.dtype} image")
        return img


    def to_message(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
        img = cv2.flip(img, 0)

        width = img.shape[1]
        height = img.shape[0]
        resolution = 0.01
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
        grid_msg.row_stride  = width * img.shape[-1] # RGBA image
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
        if self.last_lat is None or (
            self.prev_last_lat is not None and
            math.isclose(self.last_lat, self.prev_last_lat, rel_tol=0, abs_tol=self.sensitivity) and
            math.isclose(self.last_lon, self.prev_last_lon, rel_tol=0, abs_tol=self.sensitivity)
            ):
            return
        self.prev_last_lat, self.prev_last_lon = self.last_lat, self.last_lon
        last_utm_x, last_utm_y, _,_ = utm.from_latlon(self.last_lat, self.last_lon)

        img = self.call_firefox(self.last_lat, self.last_lon)

        grid_msg = self.to_message(img)
        self._map_pub.publish(grid_msg)

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
        self.last_lat, self.last_lon = msg.latitude, msg.longitude

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('google_map_cap_node', anonymous=True)
    Node().run()
