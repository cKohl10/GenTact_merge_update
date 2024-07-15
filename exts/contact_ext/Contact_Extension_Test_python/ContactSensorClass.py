# Tactile Contact Sensor Functions
# Author: Carson Kohlbrenner
# Date: 6/3/2024

from .AbstracSensorClass import AbstractSensorOperator

import numpy as np
import omni.kit.commands
import omni.ui as ui

# ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from omni.isaac.sensor import _sensor
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_children
from omni.isaac.ui.element_wrappers import CollapsableFrame
from omni.isaac.ui.ui_utils import get_style, LABEL_WIDTH
from omni.isaac.ui.element_wrappers import CollapsableFrame, IntField, DropDown, Button, StateButton
from pxr import Gf
from math import ceil

class ContactSensorOperator(AbstractSensorOperator):

    def __init__(self):
        super().__init__()
        self.parent_paths = [] # List of parent paths for each sensor
        self.sliders = [] # List of sliders for each sensor on the UI
        self.meters_per_unit = 1.00 # Unit conversion factor
        self.activated = False # Flag to determine if the sensors are active
        self.sensor_description = "Contact Sensors" # Description of the sensor type
        self.wrapped_ui_elements = [] # List of wrapped UI elements
        self.data_source = "Sim" # Data source for the sensor readings
        self.ROS_enabled = False # Flag to determine if the ROS node is connected

    # Data structure to store sensor information
    class Sensor:
        def __init__(self, name, position, radius, parent_path):
            self.name = name
            self.position = position
            self.radius = radius
            self.parent_path = parent_path
            self.path = parent_path + "/tact_sensor_" + name

    def import_sensors_fn(self):
        """
        Function that executes when the user clicks the 'Update' button
        Imports the sensor data from the CSV file and creates the sensors
        Expects the CSV file to have the following format:
        Sensor Name, X Offset, Y Offset, Z Offset, Radius, Parent Path
        """

        self.activated = True
        self._cs = _sensor.acquire_contact_sensor_interface()

        # Remove all sensors already on the robot
        message = "Removing existing sensors...\n"
        self._status_report_field.set_text(message)
        self.remove_sensors()

        message += "Sensors successfully removed\n\n"
        self._status_report_field.set_text(message)

        #Change the text of the status report field to show the import status
        path = self.config_path
        message += "Importing sensor data from '" + path + "'...\n"
        self._status_report_field.set_text(message)

        #Import the sensor data from the CSV file
        try:
            names, positions, radii, parent_paths, data = self.import_csv(path)
            self.parent_paths = parent_paths
            self.remove_sensors() # Second call to ensure all sensors are removed after parent paths are updated
            message += "File opened successfully\n"

            # Output the data to the status report field
            # message += "\n\nSensor Data:\n"
            # for i in range(len(names)):
            #     message += str(data[i]) + "\n"

        except:
            message += "Invalid file path or file format!"
            message += "\nPlease make sure the file has at least 2 sensors and is formatted correctly.\n"
            self._status_report_field.set_text(message)
            return

        self._status_report_field.set_text(message)

        # Determine the number of sensors and their positions
        num_sensors = len(data)
        self.sensors = {}
        sensor_count = 0 # Keep track of the number of sensors created successfully
        for i in range(num_sensors):

            # Create a contact sensor at the specified position
            # message += "\nCreating sensor " + str(i) + " at position " + str(positions[i]) + "...\n"
            # self._status_report_field.set_text(message)

            # Check if the parent path is valid
            if not is_prim_path_valid(parent_paths[i]):
                message += "Could not find parent path: " + parent_paths[i] + "\n"
                self._status_report_field.set_text(message)
                continue

            # Get the parent prim
            parent_prim = get_current_stage().GetPrimAtPath(parent_paths[i])
            
            # Check if the appled link has a rigidbody component (Unknown if this is necessary)
            # if not parent_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            #     message += "Parent path does not have a rigidbody component: " + parent_paths[i] + "\n"
            #     self._status_report_field.set_text(message)
            #     continue
            
            # Create the sensor
            self.create_contact_sensor(parent_paths[i], positions[i], radii[i], names[i])
            sensor_count = sensor_count + 1

        message += "\nSuccessfully created " + str(sensor_count) + " sensors\n"
        self._status_report_field.set_text(message)

        # Populate the sensor readings frame with the new sensors
        self.update_sensor_readings_frame()

    def import_csv(self, path):
        """
        Function that imports the sensor data from a CSV file
        CSV file should have the following format:
        Sensor Name, X Offset, Y Offset, Z Offset, Radius, Parent Path
        """

        try:
            data = np.genfromtxt(path, delimiter=',', skip_header=1, dtype=str)
            
            # Save the first column as a list of names, the 2-4th columns as a list of positions, and the 5th column as a list of parent paths
            names = data[:, 0]

            # Convert the positions to a list of Gf.Vec3d objects
            positions = []
            for i in range(len(data)):
                positions.append(Gf.Vec3d(float(data[i, 1]), float(data[i, 2]), float(data[i, 3])))

            radii = []
            for i in range(len(data)):
                radii.append(float(data[i, 4]))

            # Save the parent paths as a list of strings
            parent_paths = []
            for i in range(len(data)):
                parent_paths.append(data[i, 5])

            return names, positions, radii, parent_paths, data
        except:
            return None
        
    def create_contact_sensor(self, parent_path, position, radius, name):
        # Create the sensor at the specified position
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="/tact_sensor_" + name,
            parent=parent_path,
            min_threshold=0,
            max_threshold=1000000,
            color=(1, 0, 0, 1),
            radius=radius,
            translation=position,
        )

        # Add the sensor to the list of sensors
        self.sensors[name] = self.Sensor(name, position, radius, parent_path)
    
    def remove_sensors(self):
        """
        Function that removes all sensors from the robot
        """
        if len(self.parent_paths) == 0:
            return
        
        for parent_path in self.parent_paths:

            # Find all prims under the parent path that contain "tact_sensor" in their name
            try:
                parent_prim = get_current_stage().GetPrimAtPath(parent_path)
                prims = get_prim_children(parent_prim)
            except:
                self._status_report_field.set_text("Unexpected path!\n")
                return

            #self._status_report_field.set_text("Found " + str(len(prims)) + " sensors to remove\n")

            # Remove all prims found
            for prim in prims:
                if "tact_sensor" in prim.GetName():
                    omni.kit.commands.execute('DeletePrims', paths=[parent_path + "/" + prim.GetName()])
            
        self.sensors = {}

    def remove_sensors_fn(self):
        """
        Function that executes when the user clicks the 'Remove Sensors' button
        Removes all sensors from the robot
        """
        self.activated = False

        self.remove_sensors()
        self._status_report_field.set_text("All sensors removed\n\n\n If sensors remain, choose the correct configuration file and click 'Update'\n")

        # Clear the sensor readings frame
        self.update_sensor_readings_frame()

    # This function updates the sensor readings in the UI at every physics step
    def sensor_update(self, dt):
        #self._status_report_field.set_text("Updating sensor readings...\n")
        if len(self.sliders) > 0:
            slider_num = 0

            # Update the sliders with simulated values only if the data source is set to "Sim"
            if self.data_source == "Sim":
                for s in self.sensors.values():
                    #self._status_report_field.set_text("Updating sensor " + s.name + " at path " + s.path + "...\n")
                    reading = self._cs.get_sensor_reading(s.path)
                    if reading.is_valid:
                        self.sliders[slider_num].model.set_value(
                            float(reading.value) * self.meters_per_unit
                        )  # readings are in kg⋅m⋅s−2, converting to Newtons
                    else:
                        self.sliders[slider_num].model.set_value(0)

                    slider_num += 1

            # If the data soruce is set to real, make sure the values are coming from the ROS node
            elif self.data_source == "Real":
                # Check if the ROS node is connected
                if self.ROS_enabled:
                    # Get the sensor readings from the ROS node
                    rclpy.spin_once(self.touch_sub, timeout_sec=0)  # Process ROS 2 messages
                    sensor_readings = self.touch_sub.sensor_readings
                    for i in range(len(sensor_readings)):
                        self.sliders[slider_num].model.set_value(sensor_readings[i])
                        slider_num += 1



            # contacts_raw = self._cs.get_body_contact_raw_data(self.leg_paths[0])
            # if len(contacts_raw):
            #     c = contacts_raw[0]
            #     # print(c)

    def create_sensor_readings_frame(self):
        self.sensor_readings_frame = CollapsableFrame("Contact Sensor Readings", collapsed=False)

        int_field = IntField(
            "Zoom Level:",
            default_value=8,
            tooltip="Type an int or click and drag to set a new value.",
            lower_limit=1,
            upper_limit=30,
            on_value_changed_fn=self._on_int_field_value_changed_fn,
        )
        self.wrapped_ui_elements.append(int_field)

        data_source_dropdown = DropDown(
            "Data Source:",
            tooltip=" Select an option from the DropDown",
            populate_fn=self._DSD_populate_fn,
            on_selection_fn=self._DSD_item_selection,
        )
        self.wrapped_ui_elements.append(data_source_dropdown)
        data_source_dropdown.repopulate()  # This does not happen automatically, and it triggers the on_selection_fn

        # Connect to a real sensor using this button
        connect_ROS_button = StateButton(
                label="ROS Connection",
                a_text="Connect",
                b_text="Disconnect",
                tooltip="Connect to a ROS master node",
                on_a_click_fn=self.connect_ROS_fn,
                on_b_click_fn=self.disconnect_ROS_fn,
            )
        self.wrapped_ui_elements.append(connect_ROS_button)

    # def update_sensor_readings_frame(self):

    #     # Color and style for the UI elements
    #     self.sliders = []
    #     self.colors = [0xFFBBBBFF, 0xFFBBFFBB, 0xBBFFBBBB, 0xBBBBFFFF]
    #     style = {"background_color": 0xFF888888, "color": 0xFF333333, "secondary_color": self.colors[0]}
    #     #message = "There are " + str(len(self.sensors)) + " sensors\n"

    #     with self.sensor_readings_frame:
    #         # Vertical stack to hold the sensor readings in the frame
    #         with ui.VStack(style=get_style(), spacing=5, height=0):
    #             for s in self.sensors.values():
    #                 #message += "Creating reading bar for sensor " + s.name + "...\n"
    #                 with ui.HStack():
    #                     #ui.Label(s.name, width=LABEL_WIDTH, tooltip="Force in Newtons")
    #                     # ui.Spacer(height=0, width=10)
    #                     style["secondary_color"] = self.colors[0]
    #                     self.sliders.append(ui.FloatDrag(min=0.0, max=15.0, step=0.001, style=style))
    #                     self.sliders[-1].enabled = False
    #                     ui.Spacer(width=2)
    #                     self.sliders.append(ui.FloatDrag(min=0.0, max=15.0, step=0.001, style=style))
    #                     self.sliders[-1].enabled = False

    def update_sensor_readings_frame(self):

        # Color and style for the UI elements
        self.sliders = []
        self.colors = [0xFFBBBBFF, 0xFFBBFFBB, 0xBBFFBBBB, 0xBBBBFFFF]
        style = {"background_color": 0xFF888888, "color": 0xFF333333, "secondary_color": self.colors[0]}
        #message = "There are " + str(len(self.sensors)) + " sensors\n"

        sensors_per_row = self.wrapped_ui_elements[0].get_value()
        num_sensors = len(self.sensors)
        x = 0 # Keep track of the number of sensors created in GUI

        with self.sensor_readings_frame:
            # Vertical stack to hold the sensor readings in the frame
            with ui.VStack(style=get_style(), spacing=5, height=0):
                for i in range(ceil(num_sensors / sensors_per_row)):
                    #message += "Creating reading bar for sensor " + s.name + "...\n"
                    with ui.HStack():
                        style["secondary_color"] = self.colors[0]
                        for j in range(min(num_sensors-(i*sensors_per_row), sensors_per_row)):
                            self.sliders.append(ui.FloatDrag(min=0.0, max=15.0, step=0.001, style=style))
                            self.sliders[-1].enabled = False
                            ui.Spacer(width=2)


    ##############################################################################################################
    # Helper Functions
    ##############################################################################################################

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

    def _on_int_field_value_changed_fn(self, value):
        self.wrapped_ui_elements[0].set_value(value)
        self.update_sensor_readings_frame()

    def _DSD_populate_fn(self):
        # Populate the dropdown with the available data sources
        return ["Sim", "Real"]
    
    def _DSD_item_selection(self, item):
        # Update the data source string
        self.data_source = item
        self.wrapped_ui_elements[1].set_selection(item)

    def connect_ROS_fn(self):
        # Establish connection with a master ROS node, then subscribe to the data stream topic
        #rclpy.init(args=None)
        self.touch_sub = self.TouchSensorSubscriber()

        # Flag that the ROS node has been enabled
        self.ROS_enabled = True


    def disconnect_ROS_fn(self):
        # Disconnect from the ROS master node
        self.wrapped_ui_elements[2].reset()

        self.ROS_enabled = False
                            