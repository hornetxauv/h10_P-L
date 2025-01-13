import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class CompressedNode(Node):
    def __init__(self):
        super().__init__("compress_node_v2")

        self.bridge = CvBridge()

        # Create publisher
        namespace = self.get_namespace()
        self.publisher = self.create_publisher(
            # removed the / in front of self.namspace
            CompressedImage,
            f"{namespace}/rect/compressed",
            10,
        )

        # Create subscribers to raw image
        self.subscriber = self.create_subscription(
            Image,
            f"{namespace}/image_rect_color",
            self.image_callback,
            10,
        )

        # to test fps
        self.cnt = 0
        self.init_time = time.time()

    # callback function to publish the calibrated image once received
    # uses the cv2.undistort & getOptimalNewCameraMatrix functions to process
    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        compressed_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        compressed_msg.header.stamp = msg.header.stamp
        self.publisher.publish(compressed_msg)
        # self.get_logger().info(f"published compressed msg: {compressed_msg}")

        # to test fps
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
    compress_node = CompressedNode()

    rclpy.spin(compress_node)
    compress_node.destroy_node()
    rclpy.shutdown()
