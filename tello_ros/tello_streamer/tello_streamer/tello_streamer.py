import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image


class StreamerSubscriber(Node):

    def __init__(self):
        super().__init__('tello_streamer')
        self.bridge = CvBridge()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            Image,
            'drone1/image_raw',    
            self.listener_callback,
            qos_profile=qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv.imshow("frame", img)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    streamer_subscriber = StreamerSubscriber()

    rclpy.spin(streamer_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    streamer_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()