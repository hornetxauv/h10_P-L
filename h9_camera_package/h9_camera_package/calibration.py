import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo

# add back cameraInfo publisher
# import CameraInfo if want to publish calibration data


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibrator_node")

        self.declare_parameter("calibration_data_path", rclpy.Parameter.Type.STRING)
        self.calibration_data = self.read_data(
            self.get_parameter("calibration_data_path")
            .get_parameter_value()
            .string_value
        )

        self.bridge = CvBridge()

        # Create publishers
        namespace = self.get_namespace()
        self.publisher = self.create_publisher(
            CameraInfo,
            f"{namespace}/camera_info",
            10,
        )

        # Create subscribers to raw image
        self.subscriber = self.create_subscription(
            CompressedImage,
            f"{namespace}/compressed",  # sub to compressed img
            self.image_callback,
            10,
        )

        # added to test fps
        self.cnt = 0
        self.init_time = time.time()

    # callback function to publish the calibrated camera info 
    def image_callback(self, msg):
        infoMsg = self.generate_message(self.calibration_data)
        # use the time stamp of the orginal image
        infoMsg.header.stamp = msg.header.stamp
        self.publisher.publish(infoMsg)

        # added to test fps
        self.cnt += 1
        elapsed_time = time.time() - self.init_time
        if elapsed_time > 0:
            fps = self.cnt / elapsed_time
        if elapsed_time > 10:  # Reset after 10s
            self.init_time = time.time()
            self.cnt = 0
        self.get_logger().info(f"FPS: {fps}")

    def read_data(self, path):
        print("path: ", path)
        data = {
            # "name": None,
            "height": None,
            "width": None,
            "distortion_model": None,
            "d": None,
            "k": None,
            "r": None,
            "p": None,
        }

        # read the yaml file & return the contents in a dict according to CameraInfo params
        # name not included
        with open(path, "r") as file:
            contents = yaml.safe_load(file)
            # data["name"] = contents["camera_name"]
            data["height"] = contents["image_height"]
            data["width"] = contents["image_width"]
            data["distortion_model"] = contents["distortion_model"]
            data["d"] = contents["distortion_coefficients"]["data"]
            data["k"] = contents["camera_matrix"]["data"]
            data["r"] = contents["rectification_matrix"]["data"]
            data["p"] = contents["projection_matrix"]["data"]
            
        print("data", data)

        return data

    def generate_message(self, data):
        calibration_msg = CameraInfo()
        calibration_msg.height = data["height"]
        calibration_msg.width = data["width"]
        calibration_msg.distortion_model = data["distortion_model"]
        calibration_msg.d = data["d"]
        calibration_msg.k = data["k"]
        calibration_msg.r = data["r"]
        calibration_msg.p = data["p"]

        return calibration_msg


def main(args=None):
    rclpy.init(args=args)
    calibration_node = CalibrationNode()

    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()
