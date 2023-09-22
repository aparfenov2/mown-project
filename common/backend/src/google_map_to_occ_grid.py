
"""
сервис растеризует спутниковую карту Гугл, и публикует ее как сетку препятсвий (occupancy_grid) для отображения в foxglove.
Для растеризации использует Selenium и Firefox, работаюший в фоновом процессе.
input: GPS coordinates
output: occupancy_grid
"""

import cv2
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan, NavSatFix
from foxglove_msgs.msg import Grid, PackedElementField

import numpy as np
import PIL

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

        self._map_pub = rospy.Publisher('map', Grid, queue_size=1, latch=True)
        # self._map_data_pub = rospy.Publisher('map_metadata',
        #                                      MapMetaData, latch=True)
        rospy.Subscriber('fix', NavSatFix, self.gpsCallback)
        rospy.loginfo('ready')

    def call_firefox(self, lat = 43.640722, lng = -79.3811892, z = 17):
        # Toronto Waterfront Coordinates

        # Build the URL
        url = 'https://www.google.com/maps/@' + str(lat) + ',' + str(lng) + ',' + str(z) + 'z'
        # Setting up the browser window size
        browser = self.browser
        browser.set_window_size(512, 512)
        rospy.loginfo(url)
        browser.get(url)

        # # Remove omnibox
        # js_string = "var element = document.getElementById(\"omnibox container\"); element.remove();"
        # browser.execute_script(js_string)
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
        grid_msg.frame_id = "base_link"

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

    def gpsCallback(self, msg):
        img = self.call_firefox(lat=msg.latitude, lng=msg.longitude)
        grid_msg = self.to_message(img)
        self._map_pub.publish(grid_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('google_map_cap_node', anonymous=True)
    Node().run()
