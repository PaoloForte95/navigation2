#!/usr/bin/env python3

###############################################
##      Open CV and Numpy integration        ##
###############################################
# get number of humans
# publish when human detected
# distance of human from robot/camera

import os
import time
import cv2
import numpy as np
from pathlib import Path
from skimage.segmentation import flood

import torch
import rclpy
import onnxruntime
import matplotlib.pyplot as plt

#ROS2 Imports
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from det_utils import non_max_suppression, process_image, scale_boxes

class HumanDetection:
    def __init__(self, model_path):
        self.model_path = Path(model_path)
        assert self.model_path.suffix=='.onnx' and self.model_path.exists()
        
        self.detecton_threshold = 0.5
        self.segmentation_threshold = 0.01

        self.depth_scale = 0.001
        self.bridge = CvBridge()

        self.color = np.random.randint(0, 255, 3, dtype=np.uint8)

        print("Loading Onnx Model")
        self.loadOnnxModel()
        print("Done")

        self.count = 0
        self.dists_to_humans = []
        
    def loadOnnxModel(self):
        # Load Onnx model
        cuda = False
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if cuda else ["CPUExecutionProvider"]
        self.session = onnxruntime.InferenceSession(str(self.model_path), providers=providers)
        self.output_names = [x.name for x in self.session.get_outputs()]

    def processScan(self, color_msg, depth_msg, info_msg)->bool:
        if color_msg is None or depth_msg is None or info_msg is None:
            return False

        self.fx = info_msg.k[0]
        self.fy = info_msg.k[4]
        self.cx = info_msg.k[2]
        self.cy = info_msg.k[5]

        # Convert images to numpy arrays
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')*self.depth_scale
        self.color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')

        return True

    def performHumanDetection(self):
        # Person Detection
        img = process_image(self.color_image[:,:,::-1])
        pred = self.session.run(self.output_names, {self.session.get_inputs()[0].name: img})
        pred = torch.from_numpy(pred[0])
        pred_nms = non_max_suppression(prediction=pred, conf_thres=0.3, iou_thres=0.45, classes=0, agnostic=False, max_det=50,nm=0) #32 is for seg
        for i, det in enumerate(pred_nms):
            det[:, :4] = scale_boxes([480, 640], det[:, :4], [480, 640]).round()
        num_humans = 0
        self.dists_to_humans = []
        if len(det):
            for d in det:
                if d[-2]>=self.detecton_threshold and ((d[2]-d[0]) * (d[3]-d[1]))>50:
                    num_humans += 1
                    d = d[:4].to(int).tolist()
                    depth_det = self.depth_image[d[1]:d[3], d[0]:d[2]]
                    indexes = np.argwhere(depth_det==np.median(depth_det))
                    mask = np.zeros_like(self.depth_image, dtype=bool)
                    try:
                        mask[d[1]:d[3], d[0]:d[2]] = flood(depth_det, seed_point=(indexes[0,0],indexes[0,1]), connectivity=1, tolerance=self.segmentation_threshold)
                        
                        index = np.argwhere(mask).mean(axis=0).astype(int)
                        z = self.depth_image[mask].mean()
                        x = (float(index[1]) - self.cx)*z/self.fx
                        y = (float(index[0]) - self.cy)*z/self.fy
                        
                        #Segmentation Mask
                        masked_img = np.where(mask[...,None], self.color, self.color_image[:,:,:3])
                        self.color_image = cv2.addWeighted(self.color_image[:,:,:3], 0.8, masked_img, 0.2, 0)
                        
                        #Bounding Box
                        box_org = (d[0], d[1])
                        box_end = (d[2], d[3])
                        self.color_image = cv2.rectangle(self.color_image, box_org, box_end, self.color.tolist(), 2)
                        
                        #Distance Text
                        text_org = (d[0], d[1])
                        text = f'Z:{z:.2f}'
                        self.dists_to_humans.append(z)
                        self.color_image = cv2.putText(self.color_image, text, text_org, cv2.FONT_HERSHEY_SIMPLEX, 1., self.color.tolist(), 2, cv2.LINE_AA) 
                    except:
                        print("Flood Failed.")
        self.count = num_humans

    def run(self, color_msg, depth_msg, info_msg)->bool:
        is_new = self.processScan(color_msg, depth_msg, info_msg)
        if is_new:
            self.performHumanDetection()
        return is_new


class HumanDetectionRos(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        #Declare ROS2 parameters
        self.declare_parameter("namespace", "")
        self.declare_parameter("topic_name", "human_detection")
        self.declare_parameter("module_path", "src/navigation2/nav2_human_detection/src/detector/humandet.onnx")
        self.declare_parameter("rate", 0.1)
        self.declare_parameter("release_delay", 1.0)
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.module_path = self.get_parameter("module_path").get_parameter_value().string_value
        self.release_delay = self.get_parameter("release_delay").get_parameter_value().double_value
        self.publisher_ = self.create_publisher(Bool, self.namespace+'/'+ self.topic_name, 10)
        self.color_msg = None
        self.depth_msg = None
        self.info_msg = None
        self.create_subscription(Image, self.namespace+'/camera/color/image_raw', self.color_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.namespace+'/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.namespace+'/camera/color/camera_info', self.info_callback, qos_profile_sensor_data)
        self.timer_period = self.get_parameter("rate").get_parameter_value().double_value  # seconds
        print("Time Period", self.timer_period)
        self.last_seen = None
        self.humnadet = HumanDetection(self.module_path)
        self.timer = self.create_timer(self.timer_period, self.hd_callback)

    def color_callback(self, msg):
        self.color_msg = msg

    def depth_callback(self, msg):
        self.depth_msg = msg

    def info_callback(self, msg):
        self.info_msg = msg

    def hd_callback(self):
        if not self.humnadet.run(self.color_msg, self.depth_msg, self.info_msg):
            self.get_logger().warn("No frame from the RealSense camera", throttle_duration_sec=5.0)
            return
        now = self.get_clock().now()
        if self.humnadet.count > 0:
            self.last_seen = now
        msg = Bool()
        msg.data = self.last_seen is not None and (now - self.last_seen).nanoseconds * 1e-9 < self.release_delay
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    hd_node = HumanDetectionRos()
    try:
        rclpy.spin(hd_node)
    except KeyboardInterrupt:
        pass
    finally:
        hd_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()