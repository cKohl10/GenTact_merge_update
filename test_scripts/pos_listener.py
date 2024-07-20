import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import random

# Listener node
class ContactListenerNode(Node):
    def __init__(self):
        super().__init__("contact_listener_node")
        self.create_subscription(Vector3, "touch_sensor_pos", self.contact_callback, 10)

        self.pos = []

    def contact_callback(self, msg):
        print("Contact detected")
        self.pos = [msg.x, msg.y, msg.z]

def main(args=None):
    rclpy.init(args=args)
    touch_sensor_publisher = ContactListenerNode()
    touch_sensor_publisher.get_logger().info('Touch sensor publisher node started')
    rclpy.spin(touch_sensor_publisher)
    touch_sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()