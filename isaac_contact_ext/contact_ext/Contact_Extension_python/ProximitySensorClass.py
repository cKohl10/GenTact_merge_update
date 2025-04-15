# Tactile Proximity Sensor Functions
# Author: Carson Kohlbrenner
# Date: 6/3/2024

from .AbstracSensorClass import AbstractSensorOperator
from .tactile_ros import get_prim_transform

import numpy as np
import omni.kit.commands
import omni.ui as ui

import numpy as np
import carb
import omni.isaac.RangeSensorSchema as RangeSensorSchema
from scipy.spatial.transform import Rotation as R

from omni.isaac.range_sensor import _range_sensor
from omni.isaac.range_sensor._range_sensor import acquire_generic_sensor_interface
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_children, create_prim
from omni.isaac.ui.element_wrappers import CollapsableFrame
from omni.isaac.ui.ui_utils import get_style, LABEL_WIDTH
from omni.isaac.ui.element_wrappers import CollapsableFrame, IntField, DropDown, Button, StateButton
from pxr import Gf, UsdGeom, Usd, Sdf
from math import ceil

class ProximitySensorOperator(AbstractSensorOperator):

    def __init__(self):
        super().__init__()
        self.parent_paths = [] # List of parent paths for each sensor
        self.sliders = [] # List of sliders for each sensor on the UI
        self.meters_per_unit = 1.00 # Unit conversion factor
        self.activated = False # Flag to determine if the sensors are active
        self.sensor_description = "Proximity Sensors" # Description of the sensor type
        self.wrapped_ui_elements = [] # List of wrapped UI elements
        self.rotation_count = 0.0 # Counter to keep track of the number of rotations
        self.stage = get_current_stage()
        self.generic_sensors = {}

    # Data structure to store sensor information
    class Sensor:
        def __init__(self, name, position, normal, parent_path):
            self.name = name
            self.position = position
            self.parent_path = parent_path
            self.path = parent_path + "/prox_sensor_" + name
            self.normal = normal

    def import_sensors_fn(self):
        """
        Function that executes when the user clicks the 'Update' button
        Imports the sensor data from the CSV file and creates the sensors
        Expects the CSV file to have the following format:
        Sensor Name, X Offset, Y Offset, Z Offset, Radius, Parent Path
        """
        self._sensor = _range_sensor.acquire_generic_sensor_interface()

        self.activated = True

        # Apply the sensors to the robot
        self.apply_sensors()

    # This function will perform exactly like the import function without placing Proximity sensor objects to be used in sim.
    # This is practical for use with real sensors attached to the robot.
    def minimal_import_sensors_fn(self):
        """
        Function that executes when the user clicks the 'Update' button
        Imports the sensor data from the CSV file and creates the sensors
        Expects the CSV file to have the following format:
        Sensor Name, X Offset, Y Offset, Z Offset, Radius, Parent Path
        """

        self.activated = False

        # Apply the sensors to the robot
        self.apply_sensors()


    def apply_sensors(self):
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
            names, positions, normals, parent_paths, data = self.import_csv(path)
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

            # Create a Proximity sensor at the specified position
            # message += "\nCreating sensor " + str(i) + " at position " + str(positions[i]) + "...\n"
            # self._status_report_field.set_text(message)

            # Check if the parent path is valid
            if not is_prim_path_valid(parent_paths[i]):
                message += "Could not find parent path: " + parent_paths[i] + "\n"
                self._status_report_field.set_text(message)
                continue
            
            parent_prim = get_current_stage().GetPrimAtPath(parent_paths[i])

            # Create the sensor
            self.create_proximity_sensor(parent_paths[i], positions[i], normals[i], names[i], parent_prim)
            sensor_count = sensor_count + 1

        message += "\nSuccessfully created " + str(sensor_count) + " sensors\n"
        self._status_report_field.set_text(message)
        self.rotation_count += 1.0

        # if self._sensor.is_generic_sensor("/World/GenericSensor"):
        #     message += "\nSensor is generic\n"
        #     self._status_report_field.set_text(message)

        # Populate the sensor readings frame with the new sensors
        self.update_sensor_readings_frame()

        # self._editor_event_subscription = (
        #     omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_editor_step)
        # )

    def import_csv(self, path):
        """
        Function that imports the sensor data from a CSV file
        CSV file should have the following format:
        Sensor Name, X Offset, Y Offset, Z Offset, Norm X, Norm Y, Norm Z, Radius, Parent Path
        """

        try:
            data = np.genfromtxt(path, delimiter=',', skip_header=1, dtype=str)
            
            # Save the first column as a list of names, the 2-4th columns as a list of positions, and the 5th column as a list of parent paths
            names = data[:, 0]

            # Convert the positions to a list of Gf.Vec3d objects
            positions = []
            for i in range(len(data)):
                positions.append(Gf.Vec3d(float(data[i, 1]), float(data[i, 2]), float(data[i, 3])))

            # Convert the normals to a list of Gf.Vec3d objects
            normals = []
            for i in range(len(data)):
                normals.append(Gf.Vec3d(float(data[i, 4]), float(data[i, 5]), float(data[i, 6])))

            # Save the parent paths as a list of strings
            parent_paths = []
            for i in range(len(data)):
                parent_paths.append(data[i, 8])

            return names, positions, normals, parent_paths, data
        except:
            return None
        
    def create_proximity_sensor(self, parent_path, position, normal, name, parent_prim):
        # Create the sensor at the specified position
        # Note: the position vector is given relative to the parent's local frame. Effectively translation
        print(f"Sensor {name} Input Normal: ({normal[0]:.2f}, {normal[1]:.2f}, {normal[2]:.2f})")

        parent_orientation = get_prim_transform(parent_prim)[1]
        print(f"Parent Orientation: ({parent_orientation.GetAxis()[0]:.2f}, {parent_orientation.GetAxis()[1]:.2f}, {parent_orientation.GetAxis()[2]:.2f}) by {parent_orientation.GetAngle():.2f} degrees")

        rel_orientation = Gf.Rotation(self.vector_to_quaternion(normal)[1]) #Normal is relative to the parent's local frame
        print(f"Relative Orientation: ({rel_orientation.GetAxis()[0]:.2f}, {rel_orientation.GetAxis()[1]:.2f}, {rel_orientation.GetAxis()[2]:.2f}) by {rel_orientation.GetAngle():.2f} degrees")

    
        # orientation = rel_orientation * parent_orientation
        ##### Debugging #####set
        orientation = rel_orientation
        print(f"Final Orientation: ({orientation.GetAxis()[0]:.2f}, {orientation.GetAxis()[1]:.2f}, {orientation.GetAxis()[2]:.2f}) by {orientation.GetAngle():.2f} degrees\n")

        #orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        orientation = orientation.GetQuat()
        orientation = orientation.Normalize()
        orientation = np.array([orientation.GetReal(), orientation.GetImaginary()[0], orientation.GetImaginary()[1], orientation.GetImaginary()[2]])

        if self.activated:
            # result, prim = omni.kit.commands.execute(
            #     "RangeSensorCreateGeneric",
            #     path="/prox_sensor_" + name,
            #     parent=parent_path,
            #     min_range=0.4,
            #     max_range=100.0,
            #     draw_points=True,
            #     draw_lines=True,
            #     sampling_rate=60,
            #     )     
            # 
            self.generic_sensors[name]= RangeSensorSchema.Generic.Define(self.stage, parent_path + "/prox_sensor_" + name)

            # Streaming data bool: True if constantly streaming lidar points in (e.g. non-repeatable patterns)
            # False if only scan in a batch once and repeat it
            self.generic_sensors[name].CreateStreamingAttr().Set(True)

            # Min and max range for the sensor.  This defines the starting and stopping locations for the linetrace
            self.generic_sensors[name].CreateMinRangeAttr().Set(0.4)
            self.generic_sensors[name].CreateMaxRangeAttr().Set(100.0)

            # sampling rate for the custom data
            self.generic_sensors[name].CreateSamplingRateAttr().Set(20)

            # These attributes affect drawing the sensor in the viewport.
            # Draw Points = True will draw the actual rays in the viewport.
            self.generic_sensors[name].CreateDrawPointsAttr().Set(False)
            self.generic_sensors[name].CreateDrawLinesAttr().Set(False)

            # We set the attributes we created.  We could have just set the attributes at creation, but this was
            # more illustrative.  It's important to remember that attributes do not exist until you create them; even
            # if they are defined in the schema.
            self.generic_sensors[name].GetDrawLinesAttr().Set(True)      
        else:
            create_prim(
                prim_path=parent_path + "/prox_sensor_" + name,
                prim_type="Cone",
                translation=position,
                orientation=orientation,
                scale=np.array([0.01, 0.01, 0.02]),
                )

        # Add the sensor to the list of sensors
        self.sensors[name] = self.Sensor(name, position, normal, parent_path)
    
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
                if "prox_sensor" in prim.GetName():
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
        # #self._status_report_field.set_text("Updating sensor readings...\n")
        if len(self.sliders) > 0:
            slider_num = 0

            # Update the sliders with simulated values only if the data source is set to "Sim"
            if self.activated:
                for s in self.sensors.values():
                    reading = self._sensor.get_sensor_reading(s.path)

                    self.sliders[slider_num].model.set_value(
                            float(reading.value)
                        )  # readings are in kg⋅m⋅s−2, converting to Newtons


                    # else:
                    #     self.sliders[slider_num].model.set_value(0)

                    slider_num += 1

    def create_sensor_readings_frame(self):
        self.sensor_readings_frame = CollapsableFrame("Proximity Sensor Readings", collapsed=False)

        int_field = IntField(
            "Zoom Level:",
            default_value=8,
            tooltip="Type an int or click and drag to set a new value.",
            lower_limit=1,
            upper_limit=30,
            on_value_changed_fn=self._on_int_field_value_changed_fn,
        )
        self.wrapped_ui_elements.append(int_field)

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
                            self.sliders.append(ui.FloatDrag(min=0.0, max=1.0, step=0.001, style=style))
                            self.sliders[-1].enabled = False
                            ui.Spacer(width=2)


    ##############################################################################################################
    # Helper Functions
    ##############################################################################################################

    def _on_int_field_value_changed_fn(self, value):
        self.wrapped_ui_elements[0].set_value(value)
        self.update_sensor_readings_frame()

    def vector_to_quaternion(self, vector):
        # Normalize the vector
        # vector = vector / np.linalg.norm(vector)
        
        # # Default z-axis
        # z_axis = np.array([0, 0, 1])
        
        # # Calculate the rotation axis (cross product)
        # rotation_axis = np.cross(z_axis, vector)
        # if np.linalg.norm(rotation_axis) < 1e-6:
        #     # The vectors are parallel, so no rotation is needed or 180 degree rotation
        #     if np.dot(z_axis, vector) > 0:
        #         return R.from_quat([0, 0, 0, 1])  # No rotation
        #     else:
        #         return R.from_quat([1, 0, 0, 0])  # 180 degree rotation around x-axis
        
        # # Normalize the rotation axis
        # rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        
        # # Calculate the rotation angle (dot product)
        # rotation_angle = np.arccos(np.dot(z_axis, vector))
        
        # # Create the quaternion
        # quaternion = R.from_rotvec(rotation_angle * rotation_axis).as_quat()
        # return quaternion

        rot = Gf.Rotation(Gf.Vec3d(0,0,1), Gf.Vec3d(vector[0], vector[1], vector[2]))
        quaternion = rot.GetQuaternion()
        quaternion.Normalize()
        qt = np.array([quaternion.GetImaginary()[0], quaternion.GetImaginary()[1], quaternion.GetImaginary()[2], quaternion.GetReal()])
        return qt, quaternion
                                