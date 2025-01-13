import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

# import CameraInfo if want to publish calibration data


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibrator_node")

        self.declare_parameter("calibration_data_path", "left")
        self.calibration_data = self.read_data(
            self.get_parameter("calibration_data_path")
            .get_parameter_value()
            .string_value
        )

        self.bridge = CvBridge()

        # Create publishers
        namespace = self.get_namespace()
        self.publisher = self.create_publisher(
            # removed the / in front of self.namspace
            CompressedImage,
            f"{namespace}/calibrated/compressed",
            10,
        )

        # Create subscribers to raw image
        self.subscriber = self.create_subscription(
            CompressedImage,
            f"{namespace}/compressed",  # sub to compressed img
            lambda msg: self.image_callback(msg, self.calibration_data),
            10,
        )

        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        # https://stackoverflow.com/questions/39432322/what-does-the-getoptimalnewcameramatrix-do-in-opencv
        # this chunk hella sus docs may be outdated
        # roi is not used to crop like in the tut
        image_shape = (self.calibration_data["width"], self.calibration_data["height"])
        self.newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            self.calibration_data["k"],
            self.calibration_data["d"],
            image_shape,
            1,
            image_shape,
        )

        # uncomment the following & the btm functions to publish calibration data
        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.publish_calibration_data)

        # added to test fps
        self.cnt = 0
        self.init_time = time.time()

    # callback function to publish the calibrated image once received
    # uses the cv2.undistort & getOptimalNewCameraMatrix functions to process
    def image_callback(self, msg, data):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # rectification & projection matrix not used
        calibrated_img = cv2.undistort(
            cv_img, data["k"], data["d"], None, self.newcameramtx
        )
        calibrated_msg = self.bridge.cv2_to_compressed_imgmsg(calibrated_img)
        calibrated_msg.header.stamp = self.get_clock().now().to_msg()  # check timestamp
        self.publisher.publish(calibrated_msg)
        # self.get_logger().info(f"published calibrated img: {calibrated_msg}")

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
            # "r": None,
            # "p": None,
        }

        # read the yaml file & return the contents in a dict according to CameraInfo params
        # name not included
        with open(path, "r") as file:
            contents = yaml.safe_load(file)
            # data["name"] = contents["camera_name"]
            data["height"] = contents["image_height"]
            data["width"] = contents["image_width"]
            data["distortion_model"] = contents["distortion_model"]

            # reshaped based on the size defined in the yaml file
            data["d"] = np.array(contents["distortion_coefficients"]["data"]).reshape(
                (1, 5)
            )
            data["k"] = np.array(contents["camera_matrix"]["data"]).reshape(
                (3, 3)
            )  # intrinsic matrix
            data["r"] = np.array(contents["rectification_matrix"]["data"]).reshape(
                (3, 3)
            )
            data["p"] = np.array(contents["projection_matrix"]["data"]).reshape((3, 4))

        return data

def main(args=None):
    rclpy.init(args=args)
    calibration_node = CalibrationNode()

    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()
