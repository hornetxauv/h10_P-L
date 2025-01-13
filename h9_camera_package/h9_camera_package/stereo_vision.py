import rclpy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32

# This node subscribes to the left and right camera topics and publishes the estimated distance to a yellow object

class StereoVisionNode:
    def __init__(self):
        self.node = rclpy.create_node('stereo_vision_node')
        self.left_sub = self.node.create_subscription(
            Image, '/left/image_raw', self.left_image_callback, 10)
        self.right_sub = self.node.create_subscription(
            Image, '/right/image_raw', self.right_image_callback, 10)
        self.left_pub = self.node.create_publisher(
            CompressedImage, '/left/mask/compressed', 10)
        self.right_pub = self.node.create_publisher(
            CompressedImage, '/right/mask/compressed', 10)
        
        self.bridge = CvBridge()

    def left_image_callback(self, left_msg):
        self.left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        self.process_images()

    def right_image_callback(self, right_msg):
        self.right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        self.process_images()

    def process_images(self):
        if hasattr(self, 'left_image') and hasattr(self, 'right_image'):
            # Detect yellow objects in the left image
            yellow_mask = self.detect_yellow_object(self.left_image)

            # Compute disparity map
            disparity_map = self.compute_disparity(self.left_image, self.right_image)

            # Estimate distance to yellow object
            distance = self.estimate_distance(disparity_map, yellow_mask)

            # Publish the estimated distance
            self.publish_distance(distance)

    def detect_yellow_object(self, image):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define yellow color range in HSV
        lower_yellow = np.array([7, 122, 73])
        upper_yellow = np.array([29, 255, 255])

        # Threshold the HSV image to get only yellow colors
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        return yellow_mask

    def compute_disparity(self, left_image, right_image):
        left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16 * 5,
            blockSize=11,
            P1=8 * 3 * 11 ** 2,
            P2=32 * 3 * 11 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63
        )

        disparity_map = stereo.compute(left_gray, right_gray)
        return disparity_map

    def estimate_distance(self, disparity_map, object_mask):
        # Calculate average disparity of yellow object
                # Calculate average disparity of yellow object
        yellow_pixels = np.where(object_mask > 0)
        if yellow_pixels[0].size > 0:
            yellow_disparity = np.mean(disparity_map[yellow_pixels])
        else:
            yellow_disparity = 0  # or some other default value
        
        focal_length_mm = 3.2
        image_width_pixels = 640  # Replace with the actual image width in pixels
        sensor_width_mm = 3.674  # Replace with the actual sensor width in mm

        focal_length_pixels = (focal_length_mm * image_width_pixels) / sensor_width_mm
        # Replace with your focal length and baseline
        
        baseline = 0.1  # Baseline in meters

        # Calculate distance using disparity
        distance = (focal_length_pixels * baseline) / yellow_disparity
        return distance

    def publish_distance(self, distance):
        # Publish the estimated distance
        # (Replace '/object_distance' with your desired topic name)
        distance_msg = Float32()
        distance_msg.data = distance
        self.node.create_publisher(Float32, '/object_distance', 10).publish(distance_msg)

def main():
    rclpy.init()
    stereo_vision_node = StereoVisionNode()
    rclpy.spin(stereo_vision_node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
