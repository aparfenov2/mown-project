import pprint
import numpy as np
import os
import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
import torch.nn.functional as F

# ddrnet.pytorch deps
from pidnet import models, datasets
from pidnet.configs import config
from pidnet.configs import update_config
from pidnet.utils.utils import create_logger

class PIDNet:
    def __init__(self, config_file, model_state_file) -> None:
        class Args: pass
        args = Args()
        args.cfg = config_file
        args.opts = ""
        self.init_model(args, model_state_file)

    def init_model(self, args, model_state_file):
        update_config(config, args)

        self.logger, final_output_dir, _ = create_logger(
            config, args.cfg, 'test')
        logger = self.logger

        logger.info(pprint.pformat(args))
        logger.info(pprint.pformat(config))

        # cudnn related setting
        cudnn.benchmark = config.CUDNN.BENCHMARK
        cudnn.deterministic = config.CUDNN.DETERMINISTIC
        cudnn.enabled = config.CUDNN.ENABLED

        self.model = model = models.pidnet.get_seg_model(config, imgnet_pretrained=True)
        if config.TEST.MODEL_FILE:
            model_state_file = config.TEST.MODEL_FILE
        else:
            model_state_file = os.path.join(final_output_dir, 'best.pt')

        logger.info('=> loading model from {}'.format(model_state_file))

        pretrained_dict = torch.load(model_state_file)
        if 'state_dict' in pretrained_dict:
            pretrained_dict = pretrained_dict['state_dict']
        model_dict = model.state_dict()
        pretrained_dict = {k[6:]: v for k, v in pretrained_dict.items()
                            if k[6:] in model_dict.keys()}
        for k, _ in pretrained_dict.items():
            logger.info(
                '=> loading {} from pretrained model'.format(k))
        model_dict.update(pretrained_dict)
        model.load_state_dict(model_dict)

        model = model.cuda()
        # gpus = list(config.GPUS)
        # self.model = nn.DataParallel(model, device_ids=gpus).cuda()

        # prepare data
        test_size = (config.TEST.IMAGE_SIZE[1], config.TEST.IMAGE_SIZE[0])
        self.test_dataset = eval('datasets.'+config.DATASET.DATASET)(
                            root=config.DATASET.ROOT,
                            list_path=config.DATASET.TEST_SET,
                            num_classes=config.DATASET.NUM_CLASSES,
                            multi_scale=False,
                            flip=False,
                            ignore_label=config.TRAIN.IGNORE_LABEL,
                            base_size=config.TEST.BASE_SIZE,
                            crop_size=test_size,
                            )

    def do_inference(self, image):
        # expects: cv2 BGR8 image
        image_orig = image
        size = image.shape

        image = self.test_dataset.input_transform(image)
        image = image.transpose((2, 0, 1)) # CHW
        image = torch.tensor(image).unsqueeze(0)

        self.model.eval()
        with torch.no_grad():
            pred = self.test_dataset.single_scale_inference(
                config,
                self.model,
                image.cuda()
                )

            if pred.size()[-2] != size[0] or pred.size()[-1] != size[1]:
                pred = F.interpolate(
                    pred, size[-2:],
                    mode='bilinear', align_corners=config.MODEL.ALIGN_CORNERS
                )

            _, pred = torch.max(pred, dim=1)
            pred = pred.squeeze(0).cpu().numpy()
            img_pred, img_vis = self.visualize_result(image_orig, pred)

            return pred, img_pred, img_vis

    @staticmethod
    def colorEncode(labelmap, colors, mode='RGB'):
        labelmap = labelmap.astype('int')
        labelmap_rgb = np.zeros((labelmap.shape[0], labelmap.shape[1], 3),
                                dtype=np.uint8)
        for label in np.unique(labelmap):
            if label < 0:
                continue
            if label >= len(colors):
                continue
            labelmap_rgb += (labelmap == label)[:, :, np.newaxis] * \
                np.tile(colors[label],
                        (labelmap.shape[0], labelmap.shape[1], 1))

        if mode == 'BGR':
            return labelmap_rgb[:, :, ::-1]
        else:
            return labelmap_rgb

    @staticmethod
    def visualize_result(img, pred):
        colors  = np.array([[0, 0, 0],
                    [0, 0, 255],
                    [0, 255, 0],
                    [0, 255, 255],
                    [255, 0, 0 ],
                    [255, 0, 255 ],
                    [255, 255, 0 ],
                    [255, 255, 255 ],
                    [0, 0, 128 ],
                    [0, 128, 0 ],
                    [128, 0, 0 ],
                    [0, 128, 128 ],
                    [128, 0, 0 ],
                    [128, 0, 128 ],
                    [128, 128, 0 ],
                    [128, 128, 128 ],
                    [192, 192, 192 ],
                    ], dtype=np.uint8)

        pred = np.int32(pred) # 480, 640

        # colorize prediction
        pred_color = PIDNet.colorEncode(pred, colors).astype(np.uint8)

        im_vis = img * 0.7 + pred_color * 0.3
        im_vis = im_vis.astype(np.uint8)
        return pred_color, im_vis
