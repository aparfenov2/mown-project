# ------------------------------------------------------------------------------
# Modified based on https://github.com/HRNet/HRNet-Semantic-Segmentation
# ------------------------------------------------------------------------------

import os

import cv2
import numpy as np
from PIL import Image

import torch
from .base_dataset import BaseDataset

class Synth(BaseDataset):
    def __init__(self,
                 root,
                 list_path,
                 num_classes=19,
                 multi_scale=True,
                 flip=True,
                 ignore_label=255,
                 base_size=2048,
                 crop_size=(512, 1024),
                 scale_factor=16,
                 mean=[0.485, 0.456, 0.406],
                 std=[0.229, 0.224, 0.225],
                 bd_dilate_size=4):

        super().__init__(ignore_label, base_size,
                crop_size, scale_factor, mean, std,)

        self.root = root
        self.list_path = list_path
        self.num_classes = num_classes
        assert num_classes == 2

        self.multi_scale = multi_scale
        self.flip = flip

        self.img_list = [line.strip().split() for line in open(root+list_path)]

        self.files = self.read_files()

        BACK = 0
        ROAD = 255

        self.label_mapping = {
            BACK: 0,
            ROAD: 1
        }

        self.class_weights = None
        # self.class_weights = torch.FloatTensor([0.5, 0.5]).cuda()

        self.bd_dilate_size = bd_dilate_size

    def read_files(self):
        files = []
        if 'test' in self.list_path:
            for item in self.img_list:
                image_path = item
                name = os.path.splitext(os.path.basename(image_path[0]))[0]
                files.append({
                    "img": image_path[0],
                    "name": name,
                })
        else:
            for item in self.img_list:
                image_path, label_path = item
                name = os.path.splitext(os.path.basename(label_path))[0]
                files.append({
                    "img": image_path,
                    "label": label_path,
                    "name": name
                })
        return files

    def convert_label(self, label, inverse=False):
        temp = label.copy()
        if inverse:
            # assert np.max(label) == 1, str(np.max(label))
            label = np.zeros_like(label)
            for v, k in self.label_mapping.items():
                label[temp == k] = v
            # assert np.max(label) == 255, str(np.max(label))
        else:
            assert np.max(label) == 255, str(np.max(label))
            label = np.zeros_like(label)
            for k, v in self.label_mapping.items():
                label[temp == k] = v
            assert np.max(label) == 1, str(np.max(label))
        return label

    def __getitem__(self, index):
        item = self.files[index]
        name = item["name"]
        image = cv2.imread(os.path.join(self.root,'cityscapes',item["img"]),
                           cv2.IMREAD_COLOR)
        size = image.shape

        if 'test' in self.list_path:
            image = self.input_transform(image)
            image = image.transpose((2, 0, 1))

            return image.copy(), np.array(size), name

        label = cv2.imread(os.path.join(self.root,'cityscapes',item["label"]),
                           cv2.IMREAD_GRAYSCALE)
        label = self.convert_label(label)

        image, label, edge = self.gen_sample(image, label,
                                self.multi_scale, self.flip, edge_size=self.bd_dilate_size)

        return image.copy(), label.copy(), edge.copy(), np.array(size), name


    def single_scale_inference(self, config, model, image):
        pred = self.inference(config, model, image)
        return pred


    def save_pred(self, preds, sv_path, name, images=None):
        preds = np.asarray(np.argmax(preds.cpu(), axis=1), dtype=np.uint8)
        for i in range(preds.shape[0]):
            pred = self.convert_label(preds[i], inverse=True)
            if images is not None:
                image = images[i]
                image = image.numpy().transpose((1, 2, 0))
                image *= self.std
                image += self.mean
                image = image * 255.0
                image = image * 0.7 + pred[...,None] * 0.3
                image = image.astype(np.uint8)
            # print(image.shape, image.dtype, np.max(image))
            # print(pred.shape, pred.dtype, np.max(pred))
                save_img = Image.fromarray(image)
            else:
                save_img = Image.fromarray(pred)
            save_img.save(os.path.join(sv_path, name[i]+'.png'))



