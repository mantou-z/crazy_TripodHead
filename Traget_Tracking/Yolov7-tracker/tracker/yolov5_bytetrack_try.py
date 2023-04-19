"""
main code for track
"""
import numpy as np
import torch
import cv2
from PIL import Image
import tqdm

import argparse
import os
from time import gmtime, strftime
# from timer import Timer
import time
import yaml

from basetrack import BaseTracker  # for framework
from deepsort import DeepSORT
from bytetrack import ByteTrack
from deepmot import DeepMOT
from botsort import BoTSORT
from uavmot import UAVMOT
from strongsort import StrongSORT
from visualize import plot_tracking

# try:  # import package that outside the tracker folder  For yolo v7
#     import sys
#     sys.path.append(os.getcwd())

#     from models.common import DetectMultiBackend
#     from evaluate import evaluate
#     print('Note: running yolo v5 detector')

# except:
#     pass

import sys
sys.path.append(os.getcwd())

from models.common import DetectMultiBackend
from evaluate import evaluate
print('Note: running yolo v5 detector')

import tracker_dataloader
import trackeval


def set_basic_params(cfgs):
    global CATEGORY_DICT, DATASET_ROOT, CERTAIN_SEQS, IGNORE_SEQS, YAML_DICT
    CATEGORY_DICT = cfgs['CATEGORY_DICT']
    DATASET_ROOT = cfgs['DATASET_ROOT']
    CERTAIN_SEQS = cfgs['CERTAIN_SEQS']
    IGNORE_SEQS = cfgs['IGNORE_SEQS']
    YAML_DICT = cfgs['YAML_DICT']


# timer = Timer()
seq_fps = []  # list to store time used for every seq


def main(opts, cfgs):
    # set_basic_params(cfgs)  # NOTE: set basic path and seqs params first

    TRACKER_DICT = {
        'sort': BaseTracker,
        'deepsort': DeepSORT,
        'bytetrack': ByteTrack,
        'deepmot': DeepMOT,
        'botsort': BoTSORT,
        'uavmot': UAVMOT,
        'strongsort': StrongSORT,
    }  # dict for trackers, key: str, value: class(BaseTracker)

    # NOTE: ATTENTION: make kalman and tracker compatible
    if opts.tracker == 'botsort':
        opts.kalman_format = 'botsort'
    elif opts.tracker == 'strongsort':
        opts.kalman_format = 'strongsort'

    # NOTE: if save video, you must save image
    if opts.save_videos:
        opts.save_images = True

    """
    1. load model
    """
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = DetectMultiBackend(
        opts.model_path, device=device, dnn=False, data=None, fp16=False)
    model.eval()
    # warm up
    model.warmup(imgsz=(1, 3, 640, 640))
    """
    2. load dataset and track
    """
    current_time = time.localtime()

    cap = cv2.VideoCapture(opts.camid)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  # float
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float
    fps = cap.get(cv2.CAP_PROP_FPS)
    timestamp = time.strftime("%Y_%m_%d_%H_%M_%S", current_time)
    tracker = TRACKER_DICT[opts.tracker](opts, frame_rate=30, gamma=opts.gamma)
    # timer = Timer()
    frame_id = 0
    results = []
    while True:
        ret_val, frame = cap.read()
        if ret_val:
            out = model(frame.to(device))  # model forward
            out = out[0]  # NOTE: for yolo v7

            if len(out.shape) == 3:  # case (bs, num_obj, ...)
                # out = out.squeeze()
                # NOTE: assert batch size == 1
                out = out.squeeze(0)
                img0 = img0.squeeze(0)
            # remove some low conf detections
            out = out[out[:, 4] > 0.001]
            if opts.det_output_format == 'yolo':
                cls_conf, cls_idx = torch.max(out[:, 5:], dim=1)
                # out[:, 4] *= cls_conf  # fuse object and cls conf
                out[:, 5] = cls_idx
                out = out[:, :6]
            current_tracks = tracker.update(out, frame)
            cur_tlwh, cur_id, cur_cls = [], [], []
            for trk in current_tracks:
                bbox = trk.tlwh
                id = trk.track_id
                cls = trk.cls

                # filter low area bbox
                if bbox[2] * bbox[3] > opts.min_area:
                    cur_tlwh.append(bbox)
                    cur_id.append(id)
                    cur_cls.append(cls)
                    # results.append((frame_id + 1, id, bbox, cls))

                results.append((frame_id + 1, cur_id, cur_tlwh, cur_cls))
            # timer.toc()
            online_im = plot_tracking(
                frame, cur_tlwh, cur_id, frame_id=frame_id + 1, fps=30)
            cv2.imshow("results", online_im)
            ch = cv2.waitKey(1)


def save_results(folder_name, seq_name, results, data_type='default'):
    """
    write results to txt file

    results: list  row format: frame id, target id, box coordinate, class(optional)
    to_file: file path(optional)
    data_type: write data format
    """
    assert len(results)
    if not data_type == 'default':
        raise NotImplementedError  # TODO

    if not os.path.exists(f'./tracker/results/{folder_name}'):

        os.makedirs(f'./tracker/results/{folder_name}')

    with open(os.path.join('./tracker/results', folder_name, seq_name + '.txt'), 'w') as f:
        for frame_id, target_ids, tlwhs, clses in results:
            if data_type == 'default':

                # f.write(f'{frame_id},{target_id},{tlwh[0]},{tlwh[1]},\
                #             {tlwh[2]},{tlwh[3]},{cls}\n')
                for id, tlwh, cls in zip(target_ids, tlwhs, clses):
                    f.write(
                        f'{frame_id},{id},{tlwh[0]:.2f},{tlwh[1]:.2f},{tlwh[2]:.2f},{tlwh[3]:.2f},{int(cls)}\n')
    f.close()

    return folder_name


def plot_img(img, frame_id, results, save_dir):
    """
    img: np.ndarray: (H, W, C)
    frame_id: int
    results: [tlwhs, ids, clses]
    save_dir: sr

    plot images with bboxes of a seq
    """
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    img_ = np.ascontiguousarray(np.copy(img))

    tlwhs, ids, clses = results[0], results[1], results[2]
    for tlwh, id, cls in zip(tlwhs, ids, clses):

        # convert tlwh to tlbr
        tlbr = tuple([int(tlwh[0]), int(tlwh[1]), int(
            tlwh[0] + tlwh[2]), int(tlwh[1] + tlwh[3])])
        # draw a rect
        cv2.rectangle(img_, tlbr[:2], tlbr[2:], get_color(id), thickness=3, )
        # note the id and cls
        text = f'{CATEGORY_DICT[cls]}-{id}'
        cv2.putText(img_, text, (tlbr[0], tlbr[1]), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,
                    color=(255, 164, 0), thickness=2)

    cv2.imwrite(os.path.join(save_dir, f'{frame_id:05d}.jpg'), img_)


def save_videos(seq_names):
    """
    convert imgs to a video

    seq_names: List[str] or str, seqs that will be generated
    """
    if not isinstance(seq_names, list):
        seq_names = [seq_names]

    for seq in seq_names:
        images_path = os.path.join(DATASET_ROOT, 'reuslt_images', seq)
        images_name = sorted(os.listdir(images_path))

        to_video_path = os.path.join(images_path, '../', seq + '.mp4')
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")

        img0 = Image.open(os.path.join(images_path, images_name[0]))
        vw = cv2.VideoWriter(to_video_path, fourcc, 15, img0.size)

        for img in images_name:
            if img.endswith('.jpg'):
                frame = cv2.imread(os.path.join(images_path, img))
                vw.write(frame)

    print('Save videos Done!!')


def get_color(idx):
    """
    aux func for plot_seq
    get a unique color for each id
    """
    idx = idx * 3
    color = ((37 * idx) % 255, (17 * idx) % 255, (29 * idx) % 255)

    return color


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('--dataset', type=str, default='visdrone',
                        help='visdrone, visdrone_car, uavdt or mot')
    parser.add_argument('--data_format', type=str,
                        default='origin', help='format of reading dataset')
    parser.add_argument('--det_output_format', type=str, default='yolo',
                        help='data format of output of detector, yolo or other')

    parser.add_argument('--tracker', type=str,
                        default='bytetrack', help='sort, deepsort, etc')

    parser.add_argument('--model_path', type=str,
                        default=None, help='model path')

    parser.add_argument('--img_size', nargs='+', type=int,
                        default=[1280, 1280], help='[train, test] image sizes')

    parser.add_argument('--camid', type=str, default='0', help='which camera')

    """For tracker"""
    # model path
    parser.add_argument('--reid_model_path', type=str,
                        default='./weights/ckpt.t7', help='path for reid model path')
    parser.add_argument('--dhn_path', type=str,
                        default='./weights/DHN.pth', help='path of DHN path for DeepMOT')

    # threshs
    parser.add_argument('--conf_thresh', type=float,
                        default=0.5, help='filter tracks')
    parser.add_argument('--nms_thresh', type=float,
                        default=0.7, help='thresh for NMS')
    parser.add_argument('--iou_thresh', type=float,
                        default=0.5, help='IOU thresh to filter tracks')

    # other options
    parser.add_argument('--track_buffer', type=int,
                        default=30, help='tracking buffer')
    parser.add_argument('--gamma', type=float, default=0.1,
                        help='param to control fusing motion and apperance dist')
    parser.add_argument('--kalman_format', type=str, default='default',
                        help='use what kind of Kalman, default, naive, strongsort or bot-sort like')
    parser.add_argument('--min_area', type=float, default=150,
                        help='use to filter small bboxs')

    parser.add_argument('--save_images', action='store_true',
                        help='save tracking results (image)')
    parser.add_argument('--save_videos', action='store_true',
                        help='save tracking results (video)')

    # detect per several frames
    parser.add_argument('--detect_per_frame', type=int,
                        default=1, help='choose how many frames per detect')

    parser.add_argument('--track_eval', type=bool,
                        default=True, help='Use TrackEval to evaluate')

    opts = parser.parse_args()

    # NOTE: read path of datasets, sequences and TrackEval configs
    with open(f'./tracker/config_files/{opts.dataset}.yaml', 'r') as f:
        cfgs = yaml.load(f, Loader=yaml.FullLoader)
    main(opts, cfgs)
