# ROS 2
import rclpy
from rclpy.node import Node

#ROS 2 Msgs
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from tactile_msgs.srv import IndexToPos
from geometry_msgs.msg import Vector3

# USD
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Usd, UsdGeom


# This subscriber connects to a physical sensor and displays the readings in the UI
class TouchSensorSubscriber(Node):
    def __init__(self):
        super().__init__('touch_sensor_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'touch_sensor_val',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.sensor_readings = []

    def listener_callback(self, msg):
        self.sensor_readings = msg.data

# This servicer provides the contact locations of the sensors when a given sensor index is requested
class ContactLocationService(Node):
    def __init__(self):
        super().__init__('contact_location_service')
        self.srv = self.create_service(IndexToPos, 'index_to_pos', self.index_to_pos_callback)
        self.sensors = {}

    def index_to_pos_callback(self, request, response):
        
        # Get the sensor with the matching index
        try:    
            requested_sensor = self.sensors[request.index]
        except:
            self.get_logger().info("Sensor requested (%s) is nonexistant!" % request.index)
            response.position = Vector3()
            response.position.x = 0.0
            response.position.y = 0.0
            response.position.z = -1.0
            return response

        # Get the prim at the sensor's path
        prim = get_current_stage().GetPrimAtPath(requested_sensor.path)
        
        if not prim:
            self.get_logger().info("Prim is not found")
            response.position = Vector3()
            response.position.x = 0.0
            response.position.y = 0.0
            response.position.z = -1.0
            return response

        # Create an XformCache object for efficient computation of world transformations
        xformCache = UsdGeom.XformCache(Usd.TimeCode.Default())

        # Get the world transformation matrix for the prim
        world_transform = xformCache.GetLocalToWorldTransform(prim)
        
        # Extract the translation component from the world transformation matrix
        translation = world_transform.ExtractTranslation()

        # translation, orientation = prim.get_world_pose()

        response.position = Vector3()
        response.position.x = translation[0]
        response.position.y = translation[1]
        response.position.z = translation[2]
        return response
    
    # This only needs to be called when the main sensor list is updated so the servicer can also keep track of the sensors
    def update_sensor_list(self, sensors):
        self.sensors = sensors

# This publisher publishes the indices of the sensors that are currently in contact with an object
# Note: The sensors are indexed as strings, but this publisher will only handle indices that are represented as integers
class ContactListPublisher(Node):
    def __init__(self):
        super().__init__('contact_list_publisher')
        self.publisher = self.create_publisher(Int16MultiArray, 'contact_list', 10)

    def publish_contact_list(self, contact_list):
        msg = Int16MultiArray()
        msg.data = contact_list
        self.publisher.publish(msg)
        #self.get_logger().info(f'Published contact list: {msg.data}')
        
    