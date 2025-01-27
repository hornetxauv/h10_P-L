import cv2
import cv_bridge
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image

# This node subscribes to a topic and writes it to a video file
# Used to create dataset for training


class VideoWriterNode(Node):
    def __init__(self):
        super().__init__("video_writer_node")

        # self.declare_parameter("topic", "/bottom/rect/image")
        self.declare_parameter("topic", "/left/image_raw")
        topic = self.get_parameter("topic").get_parameter_value().string_value

        # Image
        self.subscription = self.create_subscription(
            Image, topic, self.image_callback, 10
        )
        # Compressed Image
        # self.subscription = self.create_subscription(
        #     CompressedImage, topic, self.image_callback, 10
        # )

        self.video_writer = None
        self.bridge = cv_bridge.CvBridge()

        self.get_logger().info(f"Subscribed to {topic}")

    def cb_params(self, data):
        for parameter in data:
            if parameter.name == "name":
                if parameter.type_ == Parameter.Type.STRING:
                    self.name = parameter.value
            self.get_logger().warn("parameter changed... {}".format(self.name))
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        try:
            self.get_logger().info('image_callback')
            # Convert (compressed) image to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            if len(image.shape) == 2:
                image = cv2.cvtColor(image, cv2.COLOR_BayerGB2BGR)

            # # rotate image (if needed)
            # image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # # resize image (if needed)
            # image = cv2.resize(image, (640, 480))

            # Initialize video writer if not already done
            if self.video_writer is None:
                width = 640
                height = 480
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self.video_writer = cv2.VideoWriter(
                    "output.mp4", fourcc, 30, (width, height)
                )

            # Write frame to video file
            self.video_writer.write(image)

        except Exception as e:
            self.get_logger().error("Error processing image: %s" % str(e))


def main(args=None):
    rclpy.init(args=args)
    node = VideoWriterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
