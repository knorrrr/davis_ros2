import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy import qos
from ev_msgs.msg import Event
from ev_msgs.msg import EventArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class EventViewerNode(Node):
    def __init__(self):
        super().__init__('event_viewer')
        self.get_logger().info('Initialized event viewer')
        self.bridge  = CvBridge()
        self.event_sub = self.create_subscription(EventArray, "~/input/event", self.callback, qos.qos_profile_sensor_data)
        self.publisher = self.create_publisher(Image, "~/output/image", 1)
        self.timer     = self.create_timer(0.04, self.publish_callback)
        self.events_store = []
        self.img_height = 0
        self.img_width  = 0

    def callback(self, msg):
        self.img_height = msg.height
        self.img_width = msg.width
        for event in msg.events:
            self.events_store.append(event)

    def publish_callback(self):
        if (len(self.events_store) > 0 and self.img_height > 0 and self.img_width > 0):
            black_img = np.zeros((self.img_height, self.img_width, 3), np.uint8)
            while (len(self.events_store) > 0):
                ev = self.events_store.pop(0)
                if ev.polarity:
                    black_img[ev.y][ev.x][2] = 255
                else:
                    black_img[ev.y][ev.x][0] = 255
            pub_msg=self.bridge.cv2_to_imgmsg(black_img)
            self.publisher.publish(pub_msg)
def main():
    rclpy.init()
    node = EventViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
