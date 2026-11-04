#!/usr/bin/env python3

import cv2
import onnxruntime
import numpy as np
import rclpy

from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8

class WeatherDetection:
    def __init__(self, model_path:str="image_based_weather_detection.onnx"):
        self._image_size = 456
        self._ort_session = onnxruntime.InferenceSession(model_path)
        self._labels = {
                            0: "Clear",
                            1: "Fog",
                            2: "Rain",
                            3: "Snow",
                       }
        self._mean = np.array([0.485, 0.456, 0.406])
        self._std = np.array([0.229, 0.224, 0.225])

    def preprocess_image(self, image):
        '''
        input:
            img: numpy array (h, w, 3) in bgr format
        '''

        h, w, _ = image.shape
        if h<=w:
            h_new = self._image_size
            w_new = int(w * (h_new / h))
        else:
            w_new = self._image_size
            h_new = int(h * (w_new / w))

        image = cv2.resize(image, (w_new, h_new))

        x = int(w_new/2 - self._image_size/2)
        y = int(h_new/2 - self._image_size/2)

        crop_img = image[int(y):int(y+self._image_size), int(x):int(x+self._image_size)] # center crop of image
        crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)
        norm_img = (crop_img/255.0 - self._mean)/self._std
        img_array = (norm_img.transpose((2, 0, 1)))[None,:,:,:]
        return img_array

    def get_inference(self, image):
        ort_inputs = {self._ort_session.get_inputs()[0].name: image.astype(np.float32)}
        ort_outs = self._ort_session.run(None, ort_inputs)
        img_out_y = ort_outs[0]

        prob = np.exp(img_out_y[0])/(np.exp(img_out_y[0]).sum())

        preds = {
                    "Clear": prob[0],
                    "Fog": prob[1],
                    "Rain": prob[2],
                    "Snow": prob[3]
                }

        return preds

    def get_weather_condition(self, preds):
        state = max(preds, key=preds.get)
        match state:
            case "Clear":
                return 0
            case "Fog":
                return 1
            case "Rain":
                return 2
            case "Snow":
                return 3


class WeatherDetectionRos(Node):
    def __init__(self):
        super().__init__('weather_detection_node')
        #Declare ROS2 parameters
        self.declare_parameter("namespace", "")
        self.declare_parameter("topic_name", "weather_condition")
        self.declare_parameter("image_topic", "camera/color/image_raw")
        self.declare_parameter("module_path", "src/navigation2/nav2_weather_detection/src/detector/image_based_weather_detection.onnx")
        self.declare_parameter("rate", 2.0)
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.module_path = self.get_parameter("module_path").get_parameter_value().string_value
        self.publisher_ = self.create_publisher(UInt8, self.namespace+'/'+ self.topic_name, 10)
        self.subscription_ = self.create_subscription(Image, self.namespace+'/'+ self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.timer_period = self.get_parameter("rate").get_parameter_value().double_value  # seconds
        self.timer = self.create_timer(self.timer_period, self.wd_callback)
        self.bridge = CvBridge()
        self.image = None
        self.weatherdet = WeatherDetection(self.module_path)

    def image_callback(self, msg):
        self.image = msg

    def wd_callback(self):
        if self.image is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
            return
        pimg = self.weatherdet.preprocess_image(img)
        preds = self.weatherdet.get_inference(pimg)
        self.get_logger().info("Predictions: %s" % preds)
        msg = UInt8()
        msg.data = self.weatherdet.get_weather_condition(preds)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    wd_node = WeatherDetectionRos()
    rclpy.spin(wd_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()