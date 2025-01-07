from __future__ import print_function
# from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from custom_msgs.msg import GateDetection

class QualiGateListener(Node):
    def __init__(self):
        super().__init__("listener")
        self.sub_gate_detection = self.create_subscription(
            GateDetection,
            "/perc/quali_gate",
            self.gate_detected_callback,
            10)
        # self.bridge = CvBridge()

    def gate_detected_callback(self, detection):
        dx, dy, distance = detection
        print(dx, dy, distance)

def main(args=None):

    rclpy.init(args=args)
    listener = QualiGateListener()
    rclpy.spin(listener)

    # Below lines are not strictly necessary
    listener.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()
