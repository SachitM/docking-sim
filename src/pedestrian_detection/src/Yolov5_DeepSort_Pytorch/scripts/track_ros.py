#!/usr/bin/env python3

from yolov5.utils.datasets import LoadImages, LoadStreams, letterbox
from yolov5.utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords, xyxy2xywh, plot_one_box, strip_optimizer)
from yolov5.utils.torch_utils import select_device, load_classifier, time_synchronized
from deep_sort.utils.parser import get_config
from deep_sort.deep_sort import DeepSort
import argparse
import os
import platform
import shutil
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
# https://github.com/pytorch/pytorch/issues/3678
import sys
sys.path.insert(0, './yolov5')

import rospy

from sensor_msgs.msg import Image

from cam_lidar_fusion.msg import Detection, Detections

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

from rospy.numpy_msg import numpy_msg

import numpy as np

palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)


class PedestrianTracker():
    def __init__(self, opt, save_img=False):

        self.opt = opt

        self.image_sub = rospy.Subscriber("/image_raw", numpy_msg(Image), self.callback)

        self.detection_pub = rospy.Publisher("pedestrian_detections", Detections)


        out, source, weights, view_img, save_txt, imgsz = \
            opt.output, opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size
        webcam = source == '0' or source.startswith('rtsp') or source.startswith('http') or source.endswith('.txt')


        torch.cuda.empty_cache()

        # initialize deepsort
        cfg = get_config()
        cfg.merge_from_file(opt.config_deepsort)
        deepsort = DeepSort(cfg.DEEPSORT.REID_CKPT,
                            max_dist=cfg.DEEPSORT.MAX_DIST, min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
                            nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP, max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                            max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET, use_cuda=True)

        self.deepsort = deepsort
        # Initialize
        device = select_device(opt.device)
        if os.path.exists(out):
            shutil.rmtree(out)  # delete output folder
        os.makedirs(out)  # make new output folder
        half = device.type != 'cpu'  # half precision only supported on CUDA

        self.device = device
        self.half = half

        # Load model
        #google_utils.attempt_download(weights)
        model = torch.load(weights, map_location=device)['model'].float()  # load to FP32
        #model = torch.save(torch.load(weights, map_location=device), weights)  # update model if SourceChangeWarning
        # model.fuse()

        model.to(device).eval()
        if half:
            model.half()  # to FP16

        self.model = model

        self.img_size = 320
        # Set Dataloader
        # vid_path, vid_writer = None, None
        # if webcam:
        #     view_img = True
        #     cudnn.benchmark = True  # set True to speed up constant image size inference
        #     dataset = LoadStreams(source, img_size=imgsz)
        # else:
        #     view_img = True
        #     save_img = True
        #     dataset = LoadImages(source, img_size=imgsz)

        # Get names and colors
        self.names = model.module.names if hasattr(model, 'module') else model.names

        # Run inference
        t0 = time.time()

    def callback(self, im0):

        opt = self.opt
        # for frame_idx, (path, img, im0s, vid_cap) in enumerate(dataset):


        im0 = np.frombuffer(im0.data, dtype=np.uint8).reshape(512, 512, -1)

        # im0 = np.frombuffer(im0.data, dtype=np.uint8).reshape(1024, 1024, -1)
        # im0 = np.rot90(im0)
        # im0 = np.rot90(im0)
        # im0 = np.rot90(im0)
        # im0 = im0[:, :, [2, 1, 0]]
        # cv2.imshow("ff", im0)
        # cv2.waitKey()

        # print(im0[100, 100, 2])
        # print(im0[100, 100, 1])
        # print(im0[100, 100, 0])
        # return

        # img = bridge.imgmsg_to_cv2(im0, "passthrough")

        img = letterbox(im0, new_shape=self.img_size)[0]

        # cv2.imshow("ff", img)
        # cv2.waitKey()



        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # print("img", torch.max(img), torch.min(img))
        # print(img)

        # Inference
        t1 = time_synchronized()
        pred = self.model(img, augment=opt.augment)[0]

        # Apply NMS

        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t2 = time_synchronized()

        # print("Helllo")

        # Process detections
        for i, det in enumerate(pred):  # detections per image

            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class

                bbox_xywh = []
                confs = []

                # Adapt detections to deep sort input format
                for *xyxy, conf, cls in det:
                    img_h, img_w, _ = im0.shape
                    x_c, y_c, bbox_w, bbox_h = self.bbox_rel(img_w, img_h, *xyxy)
                    obj = [x_c, y_c, bbox_w, bbox_h]
                    bbox_xywh.append(obj)
                    confs.append([conf.item()])

                xywhs = torch.Tensor(bbox_xywh)
                confss = torch.Tensor(confs)

                # Pass detections to deepsort
                outputs = self.deepsort.update(xywhs, confss, im0)

                print(outputs)

                # Write MOT compliant results to file
                detections = Detections()
                for j, output in enumerate(outputs):
                    det = Detection()
                    det.left = output[0]
                    det.top = output[1]
                    det.w = output[2]
                    det.h = output[3]
                    det.identity = output[-1]
                    detections.detections.append(det)
                    # print(det.identity)
                # print("ok ")


                self.detection_pub.publish(detections)


                        # with open(txt_path, 'a') as f:
                        #     f.write(('%g ' * 10 + '\n') % (frame_idx, identity, bbox_left,
                        #             bbox_top, bbox_w, bbox_h, -1, -1, -1, -1))  # label format

            # Print time (inference + NMS)
            # print('Done. (%.3fs)' % (t2 - t1))





            # # Stream results
            # if view_img:
            #     cv2.imshow(p, im0)
            #     if cv2.waitKey(1) == ord('q'):  # q to quit
            #         raise StopIteration

            # # Save results (image with detections)
            # if save_img:
            #     if dataset.mode == 'images':
            #         cv2.imwrite(save_path, im0)
            #     else:
            #         if vid_path != save_path:  # new video
            #             vid_path = save_path
            #             if isinstance(vid_writer, cv2.VideoWriter):
            #                 vid_writer.release()  # release previous video writer

            #             fps = vid_cap.get(cv2.CAP_PROP_FPS)
            #             w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            #             h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            #             vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*opt.fourcc), fps, (w, h))
            #         vid_writer.write(im0)

    @staticmethod
    def bbox_rel(image_width, image_height,  *xyxy):
        """" Calculates the relative bounding box from absolute pixel values. """
        bbox_left = min([xyxy[0].item(), xyxy[2].item()])
        bbox_top = min([xyxy[1].item(), xyxy[3].item()])
        bbox_w = abs(xyxy[0].item() - xyxy[2].item())
        bbox_h = abs(xyxy[1].item() - xyxy[3].item())
        x_c = (bbox_left + bbox_w / 2)
        y_c = (bbox_top + bbox_h / 2)
        w = bbox_w
        h = bbox_h
        return x_c, y_c, w, h


    @staticmethod
    def compute_color_for_labels(label):
        """
        Simple function that adds fixed color depending on the class
        """
        color = [int((p * (label ** 2 - label + 1)) % 255) for p in palette]
        return tuple(color)

    @staticmethod
    def draw_boxes(img, bbox, identities=None, offset=(0,0)):
        for i, box in enumerate(bbox):
            x1, y1, x2, y2 = [int(i) for i in box]
            x1 += offset[0]
            x2 += offset[0]
            y1 += offset[1]
            y2 += offset[1]
            # box text and bar
            id = int(identities[i]) if identities is not None else 0
            color = compute_color_for_labels(id)
            label = '{}{:d}'.format("", id)
            t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 2 , 2)[0]
            cv2.rectangle(img, (x1, y1),(x2,y2), color, 3)
            cv2.rectangle(img, (x1, y1), (x1 + t_size[0] + 3, y1 + t_size[1] + 4), color, -1)
            cv2.putText(img, label, (x1, y1 + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 2, [255, 255, 255], 2)
        return img





if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='yolov5/weights/yolov5s.pt', help='model.pt path')
    parser.add_argument('--source', type=str, default='inference/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='inference/output', help='output folder')  # output folder
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.4, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--fourcc', type=str, default='mp4v', help='output video codec (verify ffmpeg support)')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    # class 0 is person
    parser.add_argument('--classes', nargs='+', type=int, default=[0], help='filter by class')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument("--config_deepsort", type=str, default="deep_sort/configs/deep_sort.yaml")
    args = parser.parse_args()
    args.img_size = check_img_size(args.img_size)
    print(args)

    with torch.no_grad():
        pd = PedestrianTracker(args)
        rospy.init_node('tracker', anonymous=True)
        rospy.spin()
