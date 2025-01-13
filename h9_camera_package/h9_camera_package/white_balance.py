import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge

class EnhanceNode(Node):
    def __init__(self):
        super().__init__('enhance_node')
        # self.dehazed_pub = self.create_publisher(CompressedImage, 'dehazed_image/compressed', 10)
        self.grayworld_pub = self.create_publisher(CompressedImage, '/gray_world/compressed', 10)

        self.compressed_image_subscription = self.create_subscription(
            Image, 
            'bottom/rect/image', 
            self.image_callback, 10)
        

        self.bridge = CvBridge()

    # Callback function for image messages
    def image_callback(self, msg):
        # Convert image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_image(image)

    # Callback function for compressed image messages
    def compressed_image_callback(self, msg):
        # Convert compressed image message to OpenCV image
        image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.process_image(image)

    # Common function for image enhancement
    def process_image(self, image):


        # Perform gray-world color correction
        gray_world_image = perform_gray_world(image)
        enhance_msg = self.bridge.cv2_to_compressed_imgmsg(gray_world_image)
        self.grayworld_pub.publish(enhance_msg)

        # Perform dark channel prior dehazing
        # dehazed_image = perform_dehazing(gray_world_image)
        # dehazed_msg = self.bridge.cv2_to_compressed_imgmsg(dehazed_image)
        # self.dehazed_pub.publish(dehazed_msg)

# def perform_dehazing(image):
#     # Convert the image to float
#     image = image.astype('float64')

#     # Define the size of the patch used for the minimum filter
#     patch_size = 15

#     # Apply minimum filter to each channel separately
#     dark_channel = np.min(cv2.split(image), axis=0)

#     # Apply minimum filter again to get the dark channel prior
#     dark_channel = cv2.erode(dark_channel, np.ones((patch_size, patch_size)))

#     # Estimate the atmospheric light
#     atmospheric_light = np.max(dark_channel)

#     # Add a constant to the atmospheric light to increase it
#     atmospheric_light += 25

#     # Normalize the image
#     normalized_image = image / atmospheric_light

#     # Clip the values to the valid range for an 8-bit image
#     normalized_image = np.clip(normalized_image * 255, 0, 255).astype('uint8')

#     return normalized_image

def perform_gray_world(image):
    # Convert image to float32 for accurate calculations
    image_float = image.astype(np.float32)
    
    # Calculate average values for each channel
    avg_r = np.mean(image_float[:, :, 0])
    avg_g = np.mean(image_float[:, :, 1])
    avg_b = np.mean(image_float[:, :, 2])

    # Calculate the average gray value
    avg_gray = (avg_r + avg_g + avg_b) / 3.0

    # Perform gray world algorithm to balance colors
    scale_factor = 1.25
    image_float[:, :, 0] *= (avg_gray / avg_r) * scale_factor
    image_float[:, :, 1] *= (avg_gray / avg_g) * scale_factor
    image_float[:, :, 2] *= (avg_gray / avg_b) * scale_factor

    # Clip the values to ensure they are within the valid range
    image_float = np.clip(image_float, 0, 255)
    
    # Convert back to uint8
    image_normalized = image_float.astype(np.uint8)
    
    return image_normalized

def main(args=None):
    rclpy.init(args=args)
    enhance_node = EnhanceNode()
    rclpy.spin(enhance_node)
    enhance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
