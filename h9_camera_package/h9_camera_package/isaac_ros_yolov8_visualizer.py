#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# This script listens for images and object detections on the image,
# then renders the output boxes on top of the image and publishes
# the result as an image message

import cv2
import cv_bridge
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray

names = {
    0: "gate",
    1: "orange-flare",
    2: "blue-flare",
    3: "red-flare",
    4: "yellow-flare",
    5: "blue-drum",
    6: "red-drum",
}

INPUT_WIDTH = 640
INPUT_HEIGHT = 480


class Yolov8Visualizer(Node):
    QUEUE_SIZE = 1
    color = (0, 255, 0)
    bbox_thickness = 2

    def __init__(self):
        super().__init__("yolov8_visualizer")
        self._bridge = cv_bridge.CvBridge()

        namespace = self.get_namespace()
        self._processed_image_pub = self.create_publisher(
            CompressedImage, f"{namespace}/yolov8_image/compressed", self.QUEUE_SIZE
        )
        self._detections_subscription = message_filters.Subscriber(
            self, Detection2DArray, f"{namespace}/detections_output"
        )
        self._image_subscription = message_filters.Subscriber(
            self, Image, f"{namespace}/image"
        )

        self.time_synchronizer = message_filters.TimeSynchronizer(
            [self._detections_subscription, self._image_subscription],
            self.QUEUE_SIZE,
            slop=1,
        )

        self.time_synchronizer.registerCallback(self.detections_callback)

    def detections_callback(self, detections_msg, img_msg: Image):
        txt_color = (255, 0, 255)
        cv2_img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        for detection in detections_msg.detections:
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            width = detection.bbox.size_x
            height = detection.bbox.size_y

            label = names[int(detection.results[0].hypothesis.class_id)]
            conf_score = detection.results[0].hypothesis.score
            label = f"{label} {conf_score:.2f}"

            x_factor = img_msg.width / INPUT_WIDTH
            y_factor = img_msg.height / INPUT_HEIGHT

            self.get_logger().info(f"({img_msg.height}, {img_msg.width})")

            left = round((center_x - width / 2) * x_factor)
            top = round((center_y - height / 2) * y_factor)
            width = round(width * x_factor)
            height = round(height * y_factor)

            min_pt = (left, top)
            max_pt = (left + width, top + height)

            lw = max(
                round((img_msg.height + img_msg.width) / 2 * 0.003), 2
            )  # line width
            tf = max(lw - 1, 1)  # font thickness
            # text width, height
            w, h = cv2.getTextSize(label, 0, fontScale=lw / 3, thickness=tf)[0]
            outside = min_pt[1] - h >= 3

            self.get_logger().info(f"{center_y}")

            cv2.rectangle(cv2_img, min_pt, max_pt, self.color, self.bbox_thickness)
            cv2.putText(
                cv2_img,
                label,
                (min_pt[0], min_pt[1] - 2 if outside else min_pt[1] + h + 2),
                0,
                lw / 3,
                txt_color,
                thickness=tf,
                lineType=cv2.LINE_AA,
            )

        processed_img = self._bridge.cv2_to_compressed_imgmsg(cv2_img)
        self._processed_image_pub.publish(processed_img)


def main():
    rclpy.init()
    rclpy.spin(Yolov8Visualizer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
