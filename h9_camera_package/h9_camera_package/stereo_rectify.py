import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

PATH = "/home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/"


class StereoCalibrationNode(Node):
    def __init__(self):
        super().__init__("calibrator_node")

        self.bridge = CvBridge()
        self.publisher_l = self.create_publisher(
            CompressedImage, "/left/calibrated/compressed", 10
        )
        self.publisher_r = self.create_publisher(
            CompressedImage, "/right/calibrated/compressed", 10
        )
        self.subscriber_l = self.create_subscription(
            CompressedImage,
            "/left/compressed",  # sub to compressed img
            lambda msg: self.image_callback(
                msg, self.calibration_data_l, self.publisher_l
            ),
            10,
        )
        self.subscriber_r = self.create_subscription(
            CompressedImage,
            "/right/compressed",  # sub to compressed img
            lambda msg: self.image_callback(
                msg, self.calibration_data_r, self.publisher_r
            ),
            10,
        )
        self.calibration_data_l = self.read_data(
            PATH + "left.yaml"
        )  # check this path with the jetson
        self.calibration_data_r = self.read_data(PATH + "right.yaml")

        image_shape = (
            self.calibration_data_l["width"],
            self.calibration_data_l["height"],
        )

        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            self.calibration_data_l["k"],
            self.calibration_data_l["d"],
            self.calibration_data_r["k"],
            self.calibration_data_r["d"],
            image_shape,
            self.calibration_data_r["p"][:3, :3],
            self.calibration_data_r["p"][:, -1],
        )
        # this should be the params needed
        # cv2.stereoRectify(
        #     camera_matrix_left,
        #     distortion_coeff_left,
        #     camera_matrix_right,
        #     distortion_coeff_right,
        #     (image_width, image_height),
        #     R1,
        #     np.array([0, 0, 0]),
        # )

        # options instead of projection mtx for undistortmap
        # self.newcameramtx_l, roi = cv2.getOptimalNewCameraMatrix(
        #     self.calibration_data_l["k"],
        #     self.calibration_data_l["d"],
        #     image_shape,
        #     1,
        #     image_shape,
        # )
        # self.newcameramtx_r, roi = cv2.getOptimalNewCameraMatrix(
        #     self.calibration_data_r["k"],
        #     self.calibration_data_r["d"],
        #     image_shape,
        #     1,
        #     image_shape,
        # )

        # not using the stereo rectify returned values
        # gives the straightest lines but a bit of the FOV is cut off
        # can replace proj mtx with newCameraMtx but lines not as str8 minor diff
        self.calibration_data_l["maps"] = cv2.initUndistortRectifyMap(
            self.calibration_data_l["k"],  # camera matrix
            self.calibration_data_l["d"],  # distortion coeff
            self.calibration_data_l["r"],  # rectification mtx/ R1
            self.calibration_data_l["p"],  # projection mtx/ newCameraMtx/ P1
            image_shape,
            cv2.CV_32FC1,
        )
        self.calibration_data_r["maps"] = cv2.initUndistortRectifyMap(
            self.calibration_data_r["k"],  # camera matrix
            self.calibration_data_r["d"],  # distortion coeff
            self.calibration_data_r["r"],  # rectification matrix
            self.calibration_data_r["p"],
            image_shape,
            cv2.CV_32FC1,
        )
        # added to test fps
        self.cnt = 0
        self.init_time = time.time()

    # callback function to publish the calibrated image once received
    # uses the cv2.undistort & getOptimalNewCameraMatrix functions to process
    def image_callback(self, msg, data, publisher):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        calibrated_img = cv2.remap(
            cv_img, data["maps"][0], data["maps"][1], cv2.INTER_LINEAR
        )

        calibrated_msg = self.bridge.cv2_to_compressed_imgmsg(calibrated_img)
        calibrated_msg.header.stamp = self.get_clock().now().to_msg()  # check timestamp
        publisher.publish(calibrated_msg)
        self.print_fps()

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

            # reshaped based on the shape defined in the yaml file
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

    def print_fps(self):
        self.cnt += 1
        elapsed_time = time.time() - self.init_time
        if elapsed_time > 0:
            fps = self.cnt / elapsed_time
        if elapsed_time > 10:  # Reset after 10s
            self.init_time = time.time()
            self.cnt = 0
        self.get_logger().info(f"FPS: {fps}")


def main(args=None):
    rclpy.init(args=args)
    calibration_node = StereoCalibrationNode()

    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()
