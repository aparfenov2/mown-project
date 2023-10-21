import os
import argparse
from pidnet.pidnet_common import PIDNet
import cv2
from tqdm import tqdm

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('cfg')
    parser.add_argument('weights')
    parser.add_argument('src_dir')
    parser.add_argument('--out-dir')
    parser.add_argument('--pred', action='store_true')
    # parser.add_argument('opts',
    #                     help="Modify config options using the command-line",
    #                     default=None,
    #                     nargs=argparse.REMAINDER)

    args = parser.parse_args()
    net = PIDNet(config_file=args.cfg, model_state_file=args.weights)

    one_file_mode = not os.path.isdir(args.src_dir)
    if one_file_mode:
        files = [args.src_dir]
    else:
        if args.out_dir is not None:
            os.makedirs(args.out_dir, exist_ok=True)
        files = [args.src_dir + '/' + fn for fn in os.listdir(args.src_dir)]

    for fn in tqdm(files):
        img = cv2.imread(fn)
        _, pred, vis = net.do_inference(img)
        if args.pred:
            vis = pred

        if one_file_mode:
            if args.out_dir is not None:
                ofn = args.out_dir
            else:
                ofn = os.path.splitext(fn)[0] + '_vis.jpg'

        else:
            if args.out_dir is not None:
                ofn = args.out_dir + '/' + os.path.basename(fn)
            else:
                ofn = os.path.splitext(fn)[0] + '_vis.jpg'

        cv2.imwrite(ofn, vis)

if __name__ == '__main__':
    main()
