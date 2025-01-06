from __future__ import print_function
import rclpy
from rclpy.node import Node
import threading

from sensor_msgs.msg import Image, CompressedImage

import cv2
from cv_bridge import CvBridge

max_cnt_width = 200
min_cnt_height = 40
min_cnt_area = 500

# https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
max_value = 255
max_value_H = 360//2

low_H = 0
high_H = 12
low_H_2 = 170
high_H_2 = 180

low_S = 68 # for red and orange
low_S_orange = 192 # for orange only
high_S = 255

low_V = 0
low_V_orange = 68 # for orange only
high_V = 160

max_colour_mode = 2
colour_mode = 0

red_dilation_cycles = 1
clahe_limit = 10

control_panel_name = 'Adjust Thresholds'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_S_name_orange = 'Low S orange'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'
low_H_name_2 = 'Low H 2'
high_H_name_2 = 'High H 2'
low_V_name_orange = 'Low V orange'

switch_colour_name = 'switch_colour'

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, control_panel_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, control_panel_name, high_H)
def on_low_H_thresh_trackbar_2(val):
    global low_H_2
    global high_H_2
    low_H_2 = val
    low_H_2 = min(high_H_2, low_H_2)
    cv2.setTrackbarPos(low_H_name_2, control_panel_name, low_H_2)
def on_high_H_thresh_trackbar_2(val):
    global low_H_2
    global high_H_2
    high_H_2 = val
    high_H_2 = max(high_H_2, low_H_2)
    cv2.setTrackbarPos(high_H_name_2, control_panel_name, high_H_2)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, control_panel_name, low_S)
def on_low_S_thresh_trackbar_orange(val):
    global low_S_orange
    global high_S
    low_S_orange = val
    low_S_orange = min(high_S-1, low_S_orange)
    cv2.setTrackbarPos(low_S_name_orange, control_panel_name, low_S_orange)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, control_panel_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, control_panel_name, low_V)
def on_low_V_thresh_trackbar_orange(val):
    global low_V_orange
    global high_V
    low_V_orange = val
    low_V_orange = min(high_V-1, low_V_orange)
    cv2.setTrackbarPos(low_V_name_orange, control_panel_name, low_V_orange)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, control_panel_name, high_V)
def on_switch_colour_button(val):
    global colour_mode
    colour_mode = val
def on_red_dilation_cycles(val):
    global red_dilation_cycles
    red_dilation_cycles = val
def on_clahe_limit(val):
    global clahe_limit
    clahe_limit = val

cv2.namedWindow(control_panel_name)
cv2.createTrackbar(low_H_name, control_panel_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, control_panel_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_H_name_2, control_panel_name , low_H_2, max_value_H, on_low_H_thresh_trackbar_2)
cv2.createTrackbar(high_H_name_2, control_panel_name , high_H_2, max_value_H, on_high_H_thresh_trackbar_2)
cv2.createTrackbar(low_S_name, control_panel_name , low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(low_S_name_orange, control_panel_name , low_S_orange, max_value, on_low_S_thresh_trackbar_orange)
cv2.createTrackbar(high_S_name, control_panel_name , high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, control_panel_name , low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(low_V_name_orange, control_panel_name , low_V_orange, max_value, on_low_V_thresh_trackbar_orange)
cv2.createTrackbar(high_V_name, control_panel_name , high_V, max_value, on_high_V_thresh_trackbar)
cv2.createTrackbar(switch_colour_name, control_panel_name , colour_mode, max_colour_mode, on_switch_colour_button)
cv2.createTrackbar("red dilation cycles", control_panel_name , red_dilation_cycles, 5, on_red_dilation_cycles)
cv2.createTrackbar("clahe limit * 10", control_panel_name , clahe_limit, 50, on_clahe_limit)

taller_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 21)) #tall and narrow kernel for the pole
tall_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 18)) #tall and narrow kernel for the pole
tallish_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 12)) #tall and narrow kernel for the pole
wide_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (12, 2)) #tall and narrow kernel for the pole
normal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)) #tall and narrow kernel for the pole

def add_other_colour(original, input, low_SV, high_SV):
    if low_H_2 != high_H_2:
        other_colour_img = cv2.inRange(original, (low_H_2, *low_SV), (high_H_2, *high_SV))
        return cv2.bitwise_or(input, other_colour_img)
    return input

def custom_open(input):
    output = cv2.erode(input, taller_kernel, iterations=1)
    return cv2.dilate(output, tallish_kernel, iterations=2)

class Detector(Node):
    is_playing = False
    original_img = None
    prev_msg = None

    def __init__(self):
        super().__init__("detector")
        self.pub_debug_img_2 = self.create_publisher(Image, "/detected/debug_img_2", 10)
        self.pub_debug_img = self.create_publisher(Image, "/detected/debug_img", 10)
        self.sub_image_feed = self.create_subscription(
            CompressedImage,
            "/left/compressed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()

        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.when_not_playing)

        input_thread = threading.Thread(target=cv2.waitKey)
        input_thread.daemon = True
        input_thread.start()

    def image_feed_callback(self, msg):
        self.process_image(msg)
        self.is_playing = True
        self.prev_msg = msg

    def when_not_playing(self):
        if self.is_playing == True:
            self.is_playing = False
            return
        self.process_image(self.prev_msg)

    def process_image(self, msg):
        if (msg == None):
            return

        # Feel free to modify this callback function, or add other functions in any way you deem fit
        # Here is sample code for converting a coloured image to gray scale using opencv
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.original_img = cv_img
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        # https://stackoverflow.com/questions/39308030/how-do-i-increase-the-contrast-of-an-image-in-python-opencv
        h, s, v = cv2.split(hsv_img)

        # Applying CLAHE to L-channel
        # feel free to try different values for the limit and grid size:
        clahe = cv2.createCLAHE(clipLimit=clahe_limit/10, tileGridSize=(8,8))
        cl = clahe.apply(v)

        # merge the CLAHE enhanced L-channel with the a and b channel
        hsv_img = cv2.merge((h,s,cl))

        if colour_mode == 0: # both colours
            both_colours = self.find_orange_and_red(hsv_img)
            self.pub_img(both_colours)
            orange_only = self.find_orange(hsv_img)
            red_only = self.find_red(hsv_img, orange_only=orange_only, orange_and_red=both_colours)
            processed_orange = self.final_process_orange(orange_only)
            processed_red = self.final_process_red(red_only)
            display_img = self.apply_contours(self.original_img.copy(), processed_orange, label="Orange", colour=(0, 69, 255))
            display_img = self.apply_contours(display_img, processed_red, label="Red", colour=(0, 0, 255))
            self.pub_img_2(display_img, encoding="bgr8")

        elif colour_mode == 1: # only orange
            orange_only = self.find_orange(hsv_img)
            self.pub_orange(orange_only)

        else:
            red_only = self.find_red(hsv_img)
            self.pub_red(red_only)

    def find_orange_and_red(self, frame):
        both_result_img = cv2.inRange(frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        both_result_img = add_other_colour(frame, both_result_img, (low_S, low_V),(high_S, high_V))
        return both_result_img

    def find_orange(self, frame):
        orange_result_img = cv2.inRange(frame, (low_H, low_S_orange, low_V_orange), (high_H, high_S, high_V))  
        return add_other_colour(frame, orange_result_img, (low_S_orange, low_V_orange),(high_S, high_V)) #add the 170-180 end of the Hue spectrum
    
    def final_process_orange(self, frame):
        return custom_open(frame)

    def pub_orange(self, frame):
        processed = self.final_process_orange(frame)
        with_cnts = self.apply_contours(self.original_img.copy(), processed, label="Orange", colour=(0, 69, 255))

        self.pub_img_2(with_cnts, "bgr8")
        self.pub_img(processed) # https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

    def find_red(self, frame, orange_only=None, orange_and_red=None):
        orange_result_img = orange_only if orange_only is not None else self.find_orange(frame)

        orange_img_for_subtraction = cv2.erode(orange_result_img, taller_kernel, iterations=1) #first erode to get ride of traces of red
        orange_img_for_subtraction = cv2.dilate(orange_img_for_subtraction, normal_kernel, iterations=3) #dilate twice to make orange bigger

        both_result_img = orange_and_red if orange_and_red is not None else self.find_orange_and_red(frame)
        both_result_img
        return cv2.subtract(both_result_img, orange_img_for_subtraction)

    def final_process_red(self, frame):
        def red_clean(input, depth):
            if depth == 0:
                return input
            morphed = cv2.dilate(input, normal_kernel, iterations=red_dilation_cycles) #dilate horizontally to save the pole
            morphed = cv2.erode(morphed, tallish_kernel, iterations=1) #erode normally to kill the spots
            return red_clean(morphed, depth-1)
        
        return red_clean(frame, 1)

    def pub_red(self, frame):
        processed = self.final_process_red(frame)
        self.pub_img(processed)
        with_cnts = self.apply_contours(self.original_img.copy(), processed, label="red", colour=(0, 0, 255))
        self.pub_img_2(with_cnts, "bgr8") # https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

    def apply_contours(self, img, mask, remove_top=True, label=None, colour=(0, 255, 0)):
        image_height = mask.shape[0]
        #find contours
        cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if cnts == None:
            return mask

        # print("1",[cv2.contourArea(i) for i in cnts])
        
        # remove contours that are too small
        cnts = list(filter(lambda c: cv2.contourArea(c) > min_cnt_area, cnts))
        # print("2",[cv2.contourArea(i) for i in cnts])
        # remove contours that are too wide or short
        def filter_by_dimensions(cnt):
            _, _, w, h = cv2.boundingRect(cnt)
            # print(w,h)
            return (w < max_cnt_width and h > min_cnt_height)
        cnts = list(filter(lambda c: filter_by_dimensions(c), cnts))
        # print("3",[cv2.contourArea(i) for i in cnts])
        # remove contours in top 30% of image
        def get_cnt_centre(cnt):
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                return (cx, cy)
        if remove_top:
            cnts = list(filter(lambda c: get_cnt_centre(c)[1] > 0.3 * image_height, cnts))
        # print("4",[cv2.contourArea(i) for i in cnts])
        # #remove contours in bottom 10% of image
        # cnts = list(filter(lambda c: get_cnt_centre(c)[1] < 0.9 * image_height, cnts))

        if len(cnts):
            def find_top_2(arr):
                first = None
                second = None
                for i in arr:
                    area_i = cv2.contourArea(i)
                    # print(area_i)
                    area_first = 0 if first is None else cv2.contourArea(first)
                    area_second = 0 if second is None else cv2.contourArea(second)
                    if area_i > area_first:
                        second = first
                        first = i
                    elif area_i == area_first:
                        second = i
                    elif area_i > area_second:
                        second = i
                return filter(lambda x:x is not None, [first, second])
            
            # cnts = [max(cnts, key=cv2.contourArea)]
            cnts = find_top_2(cnts)
            
            for cnt in cnts:
                img = cv2.drawContours(img, cnt, -1, (0,255,0), 1)
                x,y,w,h = cv2.boundingRect(cnt)
                img = cv2.rectangle(img, (x, y), (x + w, y + h), colour, 3)

                dimensionsStr = f'({w}, {h}); {str(cv2.contourArea(cnt))}'
                if label:
                    label += f" ({dimensionsStr})"
                else:
                    label = dimensionsStr
                img = cv2.putText(img, label, (x+10, y-10), 0, 1, colour, 2)
        
        return img

    def pub_img(self, frame, encoding="mono8"):
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
        self.pub_debug_img.publish(img_msg)
    
    def pub_img_2(self, frame, encoding="mono8"):
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
        self.pub_debug_img_2.publish(img_msg)
    
def main(args=None):

    rclpy.init(args=args)
    detector = Detector()
    rclpy.spin(detector)

    # Below lines are not strictly necessary
    detector.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()
