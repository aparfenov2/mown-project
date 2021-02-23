    #!/usr/bin/env python

# runs mmdetection on ros camera topics
# outputs: segmented RGBD

import matplotlib
matplotlib.use('Agg')

import os
import tempfile
import threading
# from six.moves import urllib

import PIL
import numpy as np
import cv2

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image

from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot
from mmseg.core.evaluation import get_palette


class MMDetector:
  def __init__(self, config, checkpoint, palette, device='cuda:0'):
    self.model = init_segmentor(config, checkpoint, device=device)
    self.palette = palette

  def run(self, image):
    """Runs inference on a single image.

    Args:
      image: A PIL.Image object, raw input image.

    Returns:
      resized_image: RGB image resized from original input image.
      seg_map: Segmentation map of `resized_image`.
    """
    result = inference_segmentor(self.model, image)
    return result

  def visualize(self, img, result):
    model = self.model
    if hasattr(model, 'module'):
      model = model.module
    img = model.show_result(img, result, palette=self.palette, show=False)
    return img


class MMSegmentationNode:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._last_msg = None
        self._msg_lock = threading.Lock()

        self._publish_rate = rospy.get_param('~publish_rate', 100)
        self._visualize = rospy.get_param('~visualize', True)

        rgb_input = rospy.get_param('~rgb_input', '/camera/rgb/image_color')

        rospy.Subscriber(rgb_input, Image, self._image_callback, queue_size=1)

        self.label_pub = rospy.Publisher('~segmentation', Image, queue_size=1)
        self.vis_pub = rospy.Publisher('~segmentation_vis', Image, queue_size=1)

        model_path = rospy.get_param('~checkpoint', 'mobilenetv2_coco_voctrainaug')
        config_path = rospy.get_param('~config', 'mobilenetv2_coco_voctrainaug')
        palette = rospy.get_param('~palette', 'cityscapes')
        device = rospy.get_param('~device', 'cuda:0')

        palette = get_palette(palette)
        # model_dir = tempfile.mkdtemp()
        # tf.gfile.MakeDirs(model_dir)
        # download_path = os.path.join(model_dir, _TARBALL_NAME)
        # print('Downloading model, this might take a while...')
        # urllib.request.urlretrieve(_DOWNLOAD_URL_PREFIX + _MODEL_URLS[MODEL_NAME],
        #                    download_path)
        # print('Download completed! Loading DeepLab model...')

        # self._model = deeplab.DeepLabModel(download_path)
        self._model = MMDetector(config_path, model_path, palette, device)
        print('Model loaded successfully!')


    def run(self):
        rate = rospy.Rate(self._publish_rate)

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

                # Run detection.
                result = self.detect(rgb_image)
                seg_map = result[0].astype(np.uint16)
                # print(f"seg_map.shape {seg_map.shape}")
                # rospy.logdebug("Publishing semantic labels.")
                label_msg = self._cv_bridge.cv2_to_imgmsg(seg_map, 'mono16')
                label_msg.header = msg.header
                self.label_pub.publish(label_msg)

                if self._visualize:
                    # Overlay segmentation on RGB image.
                    image = self.visualize(rgb_image, result)
                    label_color_msg = self._cv_bridge.cv2_to_imgmsg(image, 'bgr8')
                    label_color_msg.header = msg.header
                    self.vis_pub.publish(label_color_msg)

            rate.sleep()

    def detect(self, rgb_image):
        # rgb_image = PIL.Image.fromarray(rgb_image)
        seg_map = self._model.run(rgb_image)
        # resized_im, seg_map = self._model.run(rgb_image)
        # seg_map = cv2.resize(seg_map.astype(np.float32), rgb_image.size, interpolation = cv2.INTER_NEAREST).astype(np.uint16)

        return seg_map

    def visualize(self, rgb_image, seg_map, alpha = 0.6):
        # image = rgb_image.copy()
        # seg_image = label_to_color_image(seg_map.astype(
        #     np.int64)).astype(np.uint8)
        # cv2.addWeighted(seg_image, alpha, image, 1 - alpha, 0, image)
        image = self._model.visualize(rgb_image, seg_map)
        return image


    def _image_callback(self, msg):
        rospy.logdebug("Got an image.")

        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()


def main():
    rospy.init_node('mmseg_ros_node')

    node = MMSegmentationNode()
    node.run()


if __name__ == '__main__':
    main()
