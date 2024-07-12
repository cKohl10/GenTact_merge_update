import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class TouchSensorPublisher(Node):
    def __init__(self):
        super().__init__('touch_sensor_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'touch_sensor_val', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Timer set to 1 second

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [random.uniform(0.0, 100.0) for _ in range(5)]  # Generate an array of 5 random float values
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published touch sensor values: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    touch_sensor_publisher = TouchSensorPublisher()
    rclpy.spin(touch_sensor_publisher)
    touch_sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
