import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import threading

SETTINGS = {
    "LOWER_HUE": 0,
    "UPPER_HUE": 255,
    "LOWER_SATURATION": 0,
    "UPPER_SATURATION": 255,
    "LOWER_VALUE": 0,
    "UPPER_VALUE": 255,
}

class hsv_threshold(Node):
    def __init__(self):
        super().__init__("hsv_thresholder")
        self.declare_parameter(
            "settings_file_path", 
            value="/home/aa/poolTest_ws/src/camera_ws/camera/hsv_bounds/hsv_test.txt", 
        ) 
        
        # use the launch file to decalare the files & namespace
        namespace = self.get_namespace()
        self.settings_path = self.get_parameter("settings_file_path").get_parameter_value().string_value
        self.read_settings()

        # Create sliders
        threading.Thread(target=create_gui, args=[self], daemon=True).start()

        # Create publishers for debugging
        self.bgr_pub = self.create_publisher(CompressedImage, "debug/bgr/compressed", 10)
        self.hsv_pub = self.create_publisher(CompressedImage, "debug/hsv/compressed", 10)
        self.range_pub = self.create_publisher(CompressedImage, "debug/range/compressed", 10) # not sure whats this for
        
        self.mask_pub = self.create_publisher(CompressedImage, "debug/mask/compressed", 10)
        self.final_mask_pub = self.create_publisher(CompressedImage, "debug/finalmask/compressed", 10)

        self.front_image_feed = self.create_subscription(
            CompressedImage,
            "bgr/compressed", # topic published by bag
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()


    def image_feed_callback(self, msg):
        # Update the HSV bounds
        lower_bound = np.array([SETTINGS["LOWER_HUE"], SETTINGS["LOWER_SATURATION"], SETTINGS["LOWER_VALUE"]])
        upper_bound = np.array([SETTINGS["UPPER_HUE"], SETTINGS["UPPER_SATURATION"], SETTINGS["UPPER_VALUE"]])

        # bridge is from cv_bridge (for converting ROS image/CompressedImage to opencv images)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        # cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        bgr_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.bgr_pub.publish(bgr_msg)
        # blur the image to remove noise using a 9x9 kernel 
        cv_img = cv2.GaussianBlur(cv_img, (3, 3), 0)


        

        # Convert to HSV
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
        hsv_msg = self.bridge.cv2_to_compressed_imgmsg(hsv_img)
        self.hsv_pub.publish(hsv_msg)

        # Threshold the HSV image to get yellow regions
        range_cv_img = cv2.inRange(hsv_img, lower_bound, upper_bound)
        range_msg = self.bridge.cv2_to_compressed_imgmsg(range_cv_img)
        self.range_pub.publish(range_msg)

        # Segment out gate regions using the mask
        kernel1 = np.ones((9, 9), np.uint8)
        kernel2 = np.ones((3, 3), np.uint8)

        # Morphological operations to remove noise
        # eroded = cv2.erode(range_cv_img, kernel2, iterations=1)
        dilated = cv2.dilate(range_cv_img, kernel1, iterations=5)

        gate_regions = cv2.bitwise_and(cv_img, cv_img, mask=dilated)
        mask_img = self.bridge.cv2_to_compressed_imgmsg(gate_regions)
        self.mask_pub.publish(mask_img)

        # Draw Contours and Bounding Box for Gate
        self.draw_contour(dilated, cv_img)
        final_mask_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.final_mask_pub.publish(final_mask_msg)


    def draw_contour(self, gray_img, cv_img):
        # find and draw contour
        contours, hierarchy = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours with green
        cv2.drawContours(cv_img, contours, -1, (0,255,0), 2)

        # draw bounding box
        for c in contours:
            x,y,w,h = cv2.boundingRect(c)
            # draw bounding box only if area is above area threshold
            if cv2.contourArea(c) > 1:
                # draw bounding box
                cv2.rectangle(cv_img, (x,y), (x+w,y+h), (0,0,128), 3)

                # get midpoint of contour
                M = cv2.moments(c)

                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # draw centroid
                cv2.circle(cv_img, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(cv_img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (112, 232, 100), 2)
    
    def read_settings(self):
        keys = SETTINGS.keys()
        try:
            with open(self.settings_path, "r") as f:
                for key in keys:
                    val = f.readline() 
                    SETTINGS[key] = int(val)
        except FileNotFoundError:
            raise FileNotFoundError
    
    # press 's' to save the settings NOT AUTOSAVED
    def save_settings(self):
        try:
            with open(self.settings_path, "w") as f:
                for val in SETTINGS.values():
                    f.write(f"{val}\n") 
        except FileNotFoundError as e:
            print("Error", e)            


def create_gui(node):
    cv2.namedWindow('HSV Bounds')
    cv2.createTrackbar('Lower Hue', 'HSV Bounds', SETTINGS["LOWER_HUE"], 255, update_bounds)
    cv2.createTrackbar('Upper Hue', 'HSV Bounds', SETTINGS["UPPER_HUE"], 255, update_bounds)
    cv2.createTrackbar('Lower Saturation', 'HSV Bounds', SETTINGS["LOWER_SATURATION"], 255, update_bounds)
    cv2.createTrackbar('Upper Saturation', 'HSV Bounds', SETTINGS["UPPER_SATURATION"], 255, update_bounds)
    cv2.createTrackbar('Lower Value', 'HSV Bounds', SETTINGS["LOWER_VALUE"], 255, update_bounds)
    cv2.createTrackbar('Upper Value', 'HSV Bounds', SETTINGS["UPPER_VALUE"], 255, update_bounds)
    while True:
        key = cv2.waitKey(1)  
        if key == ord("s"):
            node.save_settings()

def update_bounds(val):
    try:
        SETTINGS["LOWER_HUE"] = cv2.getTrackbarPos('Lower Hue', 'HSV Bounds')
        SETTINGS["UPPER_HUE"] = cv2.getTrackbarPos('Upper Hue', 'HSV Bounds')
        SETTINGS["LOWER_SATURATION"] = cv2.getTrackbarPos('Lower Saturation', 'HSV Bounds')
        SETTINGS["UPPER_SATURATION"] = cv2.getTrackbarPos('Upper Saturation', 'HSV Bounds')
        SETTINGS["LOWER_VALUE"] = cv2.getTrackbarPos('Lower Value', 'HSV Bounds')
        SETTINGS["UPPER_VALUE"] = cv2.getTrackbarPos('Upper Value', 'HSV Bounds')
    except cv2.error:
        pass


def main(args=None):
    rclpy.init(args=args)
    hsv_thresholder = hsv_threshold()
    rclpy.spin(hsv_thresholder)
    hsv_thresholder.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
