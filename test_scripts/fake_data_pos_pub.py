import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import random

class TouchSensorPublisher(Node):
    def __init__(self):
        super().__init__('touch_sensor_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'touch_sensor_pos', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)  # Timer set to 1 second

    def timer_callback(self):
        msg = Vector3()
        # Generate a random position between -5 and 5 for all x, y, and z values
        msg.x = float(random.uniform(-0.5, 0.5))
        msg.y = float(random.uniform(-0.5, 0.5))
        msg.z = float(random.uniform(0.2, 0.5))

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published touch sensor values: {msg.x}, {msg.y}, {msg.z}')

def main(args=None):
    rclpy.init(args=args)
    touch_sensor_publisher = TouchSensorPublisher()
    touch_sensor_publisher.get_logger().info('Touch sensor publisher node started')
    rclpy.spin(touch_sensor_publisher)
    touch_sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()