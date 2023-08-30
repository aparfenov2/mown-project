# ------------------------------------------------------------------------------
# Copyright (c) Microsoft
# Licensed under the MIT License.
# Written by Ke Sun (sunk@mail.ustc.edu.cn)
# ------------------------------------------------------------------------------

import time
import collections


# ROS stuff
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
import threading
from pidnet.pidnet_common import PIDNet

class FPS:
    def __init__(self,avarageof=50):
        self.frametimestamps = collections.deque(maxlen=avarageof)
    def __call__(self):
        self.frametimestamps.append(time.time())
        if(len(self.frametimestamps) > 1):
            return len(self.frametimestamps)/(self.frametimestamps[-1]-self.frametimestamps[0])
        else:
            return 0.0

class PIDNetNode:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._last_msg = None
        self._msg_lock = threading.Lock()

        self._publish_rate = rospy.get_param('~publish_rate', 100)
        rgb_input = rospy.get_param('~rgb_input', '/camera/rgb/image_color')

        rospy.Subscriber(rgb_input, Image, self.image_callback, queue_size=1)
        self.label_pub = rospy.Publisher('~segmentation', Image, queue_size=1)
        self.vis_pub = rospy.Publisher('~segmentation_vis', Image, queue_size=1)

        config_file = rospy.get_param('~cfg', 'experiments/map/map_hrnet_ocr_w18_small_v2_512x1024_sgd_lr1e-2_wd5e-4_bs_12_epoch484.yaml')
        model_state_file = rospy.get_param('~weights', 'best.pth')
        self.net = PIDNet(config_file, model_state_file)

    def image_callback(self, msg):
        rospy.logdebug("Got an image.")

        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

    def run(self):
        rate = rospy.Rate(self._publish_rate)
        fps = FPS()
        while not rospy.is_shutdown():
            if self._msg_lock.acquire(False):
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue

            if msg is not None:
                rgb_image = self._cv_bridge.imgmsg_to_cv2(msg, "passthrough")
                _, vis_pred, vis_image = self.net.do_inference(rgb_image)

                if vis_pred is not None:
                    label_msg = self._cv_bridge.cv2_to_imgmsg(vis_pred, 'bgr8')
                    label_msg.header = msg.header
                    self.label_pub.publish(label_msg)

                if vis_image is not None:
                    label_color_msg = self._cv_bridge.cv2_to_imgmsg(vis_image, 'bgr8')
                    label_color_msg.header = msg.header
                    self.vis_pub.publish(label_color_msg)

                rospy.loginfo_throttle(15, "inference loop frequency %f", fps())

            rate.sleep()


def main():
    rospy.init_node('pidnet_py_node')

    node = PIDNetNode()
    node.run()

if __name__ == '__main__':
    main()
