import time

import rclpy
from foxglove_msgs.msg import CompressedVideo
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CompressedNode(Node):
    def __init__(self):
        super().__init__("compress_node")

        namespace = self.get_namespace()
        self.publisher = self.create_publisher(
            CompressedVideo,
            f"{namespace}/video_compressed",
            10,
        )

        self.subscriber = self.create_subscription(
            CompressedImage,
            f"{namespace}/rect/compressed",
            self.image_callback,
            10,
        )

        self.cnt = 0
        self.init_time = time.time()

    # callback function to publish the calibrated image once received
    # uses the cv2.undistort & getOptimalNewCameraMatrix functions to process
    def image_callback(self, msg: CompressedImage):
        compressed_video = CompressedVideo()
        compressed_video.timestamp = msg.header.stamp
        compressed_video.frame_id = msg.header.frame_id
        compressed_video.data = msg.data
        compressed_video.format = msg.format

        self.publisher.publish(compressed_video)

        # # to test fps
        # self.cnt += 1
        # elapsed_time = time.time() - self.init_time
        # if elapsed_time > 0:
        #     fps = self.cnt / elapsed_time
        # if elapsed_time > 10:  # Reset after 10s
        #     self.init_time = time.time()
        #     self.cnt = 0
        # self.get_logger().info(f"FPS: {fps}")


def main(args=None):
    rclpy.init(args=args)
    compress_node = CompressedNode()

    rclpy.spin(compress_node)
    compress_node.destroy_node()
    rclpy.shutdown()
