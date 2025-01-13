import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from skimage import filters

# Detect Floor tile lines and calculate angles
# Giving estimate for yaw angles

class FloorTilesNode(Node):
    def __init__(self):
        super().__init__('floor_tiles_node')

        # self.tile_img_pub = self.create_publisher(CompressedImage, 'compressed_image_topic', 10)
        self.tile_angles = self.create_publisher(Float32MultiArray, '/poolLines', 10)
        self.bottom_image_feed = self.create_subscription(
            Image,
            '/bottom/image_rect_color',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert compressed image to OpenCV image
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # clear terminal
        print("\033c")
        # if image contain green mat, return
        if self.isOnMat(img):
            print("On mat")
            return
        
        self.calculate_angle(img)
        
    def isOnMat(self, img):
        # Split the image into its color channels
        red_channel = img[:, :, 2]
        green_channel = img[:, :, 1]
        blue_channel = img[:, :, 0]
        
        # Calculate the average intensity of each color channel
        red_intensity = np.mean(red_channel)
        green_intensity = np.mean(green_channel)
        blue_intensity = np.mean(blue_channel)
        
        # Compare the intensities to classify the image
        if red_intensity > green_intensity and red_intensity > blue_intensity:
            return True #Red Bucket
        elif green_intensity > red_intensity and green_intensity > blue_intensity:
            return True #Green Mat
        else:
            return False #Blue Bucket or Blue tiles

        

    def calculate_angle(self, img):
        # Convert to grayscale
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=1.2, tileGridSize=(12,12))
        cl1 = clahe.apply(imgGray)

        # Perform Otsu adaptive thresholding
        thresholds = filters.threshold_multiotsu(cl1, classes=2)

        # Perform Thresholding
        threshold = (cl1 > thresholds[0])
        thresholded_img = threshold.astype(np.uint8) * 255

        # Perform Canny edge detection
        edges = cv2.Canny(thresholded_img, 50, 200, None, 3)
        cdstP = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        
        # Perform Probabilistic Hough line transformation
        linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, None, 50, 10)

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

            thetas = []
            for line in linesP:
                line = line[0]
                thetas.append(abs(np.arctan2(line[3]-line[1], line[2]-line[0])))

            if len(thetas) == 0:
                print("No lines detected")
                return
            rad_45 = np.radians(45)

            # Filter the thetas list to only include angles above 45 degrees
            filtered_thetas = [theta for theta in thetas if theta > rad_45]

            if filtered_thetas == []:
                print("No lines detected")
                return
            # Calculate the average of the filtered angles
            average_angle = np.mean(filtered_thetas)

            # If you want to ensure the average angle is between 0 and 90 degrees, you can do:
            average_angle = average_angle % (2*rad_45)

            angle1 = np.degrees(average_angle)
            angle2 = 90 - angle1

            # Publish the angles message
            angles_msg = Float32MultiArray()
            angles_msg.data = [angle1, angle2]
            self.tile_angles.publish(angles_msg)
            
        else:
            print("No lines detected")
        

def main(args=None):
    rclpy.init(args=args)
    floor_tiles_node = FloorTilesNode()
    rclpy.spin(floor_tiles_node)
    floor_tiles_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
