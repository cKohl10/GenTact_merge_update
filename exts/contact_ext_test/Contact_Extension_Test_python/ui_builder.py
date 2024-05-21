# This software contains source code provided by NVIDIA Corporation.
# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.timeline
import omni.ui as ui
import omni.kit.commands
import time
import os
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects.cuboid import FixedCuboid
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import is_prim_path_valid, get_all_matching_child_prims, delete_prim, get_prim_children
from omni.isaac.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
from omni.isaac.core.world import World
from omni.isaac.ui.element_wrappers import CollapsableFrame, StateButton, Button, TextBlock, StringField, DropDown
from omni.isaac.ui.element_wrappers.core_connectors import LoadButton, ResetButton
from omni.isaac.ui.ui_utils import get_style, LABEL_WIDTH
from omni.usd import StageEventType
from pxr import Sdf, UsdLux, Gf
from omni.isaac.sensor import _sensor

from .scenario import ExampleScenario


class UIBuilder:
    def __init__(self, window):

        # Window to hold the UI elements
        self.window = window

        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self.parent_paths = []
        self.sensors = {}

        # Contact Parameters
        self.meters_per_unit = 1.00

    # Data structure to store sensor information
    class Sensor:
        def __init__(self, name, position, radius, parent_path):
            self.name = name
            self.position = position
            self.radius = radius
            self.parent_path = parent_path
            self.path = parent_path + "/tact_sensor_" + name


    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            # When the user hits the stop button through the UI, they will inevitably discover edge cases where things break
            # For complete robustness, the user should resolve those edge cases here
            # In general, for extensions based off this template, there is no value to having the user click the play/stop
            # button instead of using the Load/Reset/Run buttons provided.
            #self._scenario_state_btn.reset()
            #self._scenario_state_btn.enabled = False
            pass

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        self.contact_sensor_update(step)

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._reset_extension()

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    ################################# Individual Frames ##################################################
    def create_status_report_frame(self):
        self._status_report_frame = CollapsableFrame("Status Report", collapsed=False)
        with self._status_report_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._status_report_field = TextBlock(
                    "Last UI Event",
                    num_lines=3,
                    tooltip="Prints the latest change to this UI",
                    include_copy_button=True,
                )

    def update_sensor_readings_frame(self):

        # Color and style for the UI elements
        self.sliders = []
        self.colors = [0xFFBBBBFF, 0xFFBBFFBB, 0xBBFFBBBB, 0xBBBBFFFF]
        style = {"background_color": 0xFF888888, "color": 0xFF333333, "secondary_color": self.colors[0]}
        #message = "There are " + str(len(self.sensors)) + " sensors\n"

        with self.sensor_readings_frame:
            # Vertical stack to hold the sensor readings in the frame
            with ui.VStack(style=get_style(), spacing=5, height=0):
                for s in self.sensors.values():
                    #message += "Creating reading bar for sensor " + s.name + "...\n"
                    with ui.HStack():
                        ui.Label(s.name, width=LABEL_WIDTH, tooltip="Force in Newtons")
                        # ui.Spacer(height=0, width=10)
                        style["secondary_color"] = self.colors[0]
                        self.sliders.append(ui.FloatDrag(min=0.0, max=15.0, step=0.001, style=style))
                        self.sliders[-1].enabled = False
                        ui.Spacer(width=20)

        #self._status_report_field.set_text(message)

    def create_sensor_readings_frame(self):
        self.sensor_readings_frame = CollapsableFrame("Sensor Readings", collapsed=False)

    def create_import_sensors_frame(self):
        buttons_frame = CollapsableFrame("Import Sensors", collapsed=False)

        with buttons_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                string_field = StringField(
                    "Import CSV File",
                    default_value="TactileSim/sensor_configs",
                    tooltip="Path to sensor positioning file",
                    read_only=False,
                    multiline_okay=False,
                    on_value_changed_fn=self._on_string_field_value_changed_fn,
                    use_folder_picker=True,
                    #item_filter_fn=is_usd_or_python_path,
                )
                self.wrapped_ui_elements.append(string_field)

                dropdown = DropDown(
                    "Config Options",
                    tooltip=" Select an option from the DropDown",
                    populate_fn=self.dropdown_populate_fn,
                    on_selection_fn=self._on_dropdown_item_selection,
                )
                self.wrapped_ui_elements.append(dropdown)
                dropdown.repopulate()  # This does not happen automatically, and it triggers the on_selection_fn

                button = Button(
                    "Refresh Sensor Positions",
                    "Update",
                    tooltip="Reread the data from the specified file path to update sensors",
                    on_click_fn=self.import_sensors_fn,
                )
                self.wrapped_ui_elements.append(button)

                self._status_report_field = TextBlock(
                    "Import Status",
                    num_lines=10,
                    tooltip="Outputs the status of the import process",
                    include_copy_button=True,
                )

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """

        self._cs = _sensor.acquire_contact_sensor_interface()

        self.create_import_sensors_frame()

        self.create_sensor_readings_frame()

        #self.create_status_report_frame()

    ############################## Import Frame Functions ########################################
    def dropdown_populate_fn(self):
        """
        Function that populates the dropdown with options
        Returns all the files in the directory specified by the string field
        """
        options = []

        # Get the path from the string field
        path = self.wrapped_ui_elements[0].get_value()

        # Get all the files in the directory
        try:
            options = os.listdir(path)
        except:
            options = []

        return options
    
    def import_sensors_fn(self):
        """
        Function that executes when the user clicks the 'Refresh Sensors' button
        """

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
            self._status_report_field.set_text(message)
            return

        self._status_report_field.set_text(message)

        # Determine the number of sensors and their positions
        num_sensors = len(data)
        for i in range(num_sensors):

            # Create a contact sensor at the specified position
            # message += "\nCreating sensor " + str(i) + " at position " + str(positions[i]) + "...\n"
            # self._status_report_field.set_text(message)
            self.create_sensor(parent_paths[i], positions[i], radii[i], names[i])

        message += "\nSuccessfully created " + str(num_sensors) + " sensors\n"
        self._status_report_field.set_text(message)

        # Populate the sensor readings frame with the new sensors
        self.update_sensor_readings_frame()



    # This function breaks down the CSV file into its components. Make sure the CSV file is formatted correctly
    #
    # The CSV file should be formatted as follows:
    #   - The first row should contain the names of the sensors
    #   - The second row should contain the x, y, and z positions of the sensors
    #   - The third row should contain the radii of the sensors
    #   - The fourth row should contain the parent paths of the sensors
    #
    def import_csv(self, path):
        """
        Function that imports data from a CSV file

        Args:
            path (str): The path to the CSV file
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
        
    def create_sensor(self, parent_path, position, radius, name):
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="/tact_sensor_" + name,
            parent=parent_path,
            min_threshold=0,
            max_threshold=10000000,
            color=(1, 0, 0, 1),
            radius=radius,
            sensor_period=-1,
            translation=position,
            visualize=True,
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
            # For all "IsaacContactSensor" objects with "tact_sensor" in their name under the parent path, remove them
            #omni.kit.commands.execute('DeletePrims', paths=[parent_path + "/tact_sensor"])

            # Find all prims under the parent path that contain "tact_sensor" in their name
            parent_prim = get_current_stage().GetPrimAtPath(parent_path)
            prims = get_prim_children(parent_prim)

            #self._status_report_field.set_text("Found " + str(len(prims)) + " sensors to remove\n")

            # Remove all prims found
            for prim in prims:
                if "tact_sensor" in prim.GetName():
                    omni.kit.commands.execute('DeletePrims', paths=[parent_path + "/" + prim.GetName()])

    def _on_string_field_value_changed_fn(self, value):
        """
        Function that executes when the user changes the value of the string field
        Sets the value of the string field to what the user entered
        """
        self.wrapped_ui_elements[0].set_value(value)

    def _on_dropdown_item_selection(self, item):
        """
        Function that executes when the user selects an item from the dropdown
        Sets the value of the dropdown to the item the user selected
        """
        self.config_path = self.wrapped_ui_elements[0].get_value() + "/" + item
        pass

    ######################################################################################
    # Contact updates
    ######################################################################################
    # This function updates the sensor readings in the UI at every physics step
    def contact_sensor_update(self, dt):
        #self._status_report_field.set_text("Updating sensor readings...\n")
        if len(self.sliders) > 0:
            slider_num = 0
            for s in self.sensors.values():
                reading = self._cs.get_sensor_reading(s.path)
                if reading.is_valid:
                    self.sliders[slider_num].model.set_value(
                        float(reading.value) * self.meters_per_unit
                    )  # readings are in kg⋅m⋅s−2, converting to Newtons
                else:
                    self.sliders[slider_num].model.set_value(0)

                slider_num += 1
            # contacts_raw = self._cs.get_body_contact_raw_data(self.leg_paths[0])
            # if len(contacts_raw):
            #     c = contacts_raw[0]
            #     # print(c)


    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    # def _on_init(self):
    #     self._articulation = None
    #     self._cuboid = None
    #     self._scenario = ExampleScenario()

    # def _add_light_to_stage(self):
    #     """
    #     A new stage does not have a light by default.  This function creates a spherical light
    #     """
    #     sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
    #     sphereLight.CreateRadiusAttr(2)
    #     sphereLight.CreateIntensityAttr(100000)
    #     XFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])

    # def _setup_scene(self):
    #     """
    #     This function is attached to the Load Button as the setup_scene_fn callback.
    #     On pressing the Load Button, a new instance of World() is created and then this function is called.
    #     The user should now load their assets onto the stage and add them to the World Scene.

    #     In this example, a new stage is loaded explicitly, and all assets are reloaded.
    #     If the user is relying on hot-reloading and does not want to reload assets every time,
    #     they may perform a check here to see if their desired assets are already on the stage,
    #     and avoid loading anything if they are.  In this case, the user would still need to add
    #     their assets to the World (which has low overhead).  See commented code section in this function.
    #     """
    #     # Load the UR10e
    #     robot_prim_path = "/ur10e"
    #     path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"

    #     # Do not reload assets when hot reloading.  This should only be done while extension is under development.
    #     # if not is_prim_path_valid(robot_prim_path):
    #     #     create_new_stage()
    #     #     add_reference_to_stage(path_to_robot_usd, robot_prim_path)
    #     # else:
    #     #     print("Robot already on Stage")

    #     create_new_stage()
    #     self._add_light_to_stage()
    #     add_reference_to_stage(path_to_robot_usd, robot_prim_path)

    #     # Create a cuboid
    #     self._cuboid = FixedCuboid(
    #         "/Scenario/cuboid", position=np.array([0.3, 0.3, 0.5]), size=0.05, color=np.array([255, 0, 0])
    #     )

    #     self._articulation = Articulation(robot_prim_path)

    #     # Add user-loaded objects to the World
    #     world = World.instance()
    #     world.scene.add(self._articulation)
    #     world.scene.add(self._cuboid)

    # def _setup_scenario(self):
    #     """
    #     This function is attached to the Load Button as the setup_post_load_fn callback.
    #     The user may assume that their assets have been loaded by their setup_scene_fn callback, that
    #     their objects are properly initialized, and that the timeline is paused on timestep 0.

    #     In this example, a scenario is initialized which will move each robot joint one at a time in a loop while moving the
    #     provided prim in a circle around the robot.
    #     """
    #     self._reset_scenario()

    #     # UI management
    #     self._scenario_state_btn.reset()
    #     self._scenario_state_btn.enabled = True
    #     self._reset_btn.enabled = True

    # def _reset_scenario(self):
    #     self._scenario.teardown_scenario()
    #     self._scenario.setup_scenario(self._articulation, self._cuboid)

    # def _on_post_reset_btn(self):
    #     """
    #     This function is attached to the Reset Button as the post_reset_fn callback.
    #     The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

    #     They may also assume that objects that were added to the World.Scene have been moved to their default positions.
    #     I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
    #     """
    #     self._reset_scenario()

    #     # UI management
    #     self._scenario_state_btn.reset()
    #     self._scenario_state_btn.enabled = True

    # def _update_scenario(self, step: float):
    #     """This function is attached to the Run Scenario StateButton.
    #     This function was passed in as the physics_callback_fn argument.
    #     This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
    #     When the b_text "STOP" is pressed, the physics callback is removed.

    #     Args:
    #         step (float): The dt of the current physics step
    #     """
    #     self._scenario.update_scenario(step)

    # def _on_run_scenario_a_text(self):
    #     """
    #     This function is attached to the Run Scenario StateButton.
    #     This function was passed in as the on_a_click_fn argument.
    #     It is called when the StateButton is clicked while saying a_text "RUN".

    #     This function simply plays the timeline, which means that physics steps will start happening.  After the world is loaded or reset,
    #     the timeline is paused, which means that no physics steps will occur until the user makes it play either programmatically or
    #     through the left-hand UI toolbar.
    #     """
    #     self._timeline.play()

    # def _on_run_scenario_b_text(self):
    #     """
    #     This function is attached to the Run Scenario StateButton.
    #     This function was passed in as the on_b_click_fn argument.
    #     It is called when the StateButton is clicked while saying a_text "STOP"

    #     Pausing the timeline on b_text is not strictly necessary for this example to run.
    #     Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
    #     the robot will stop getting new commands and the cube will stop updating without needing to
    #     pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
    #     forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
    #     this example prettier, but if curious, the user should observe what happens when this line is removed.
    #     """
    #     self._timeline.pause()

    # def _reset_extension(self):
    #     """This is called when the user opens a new stage from self.on_stage_event().
    #     All state should be reset.
    #     """
    #     self._on_init()
    #     self._reset_ui()

    # def _reset_ui(self):
    #     self._scenario_state_btn.reset()
    #     self._scenario_state_btn.enabled = False
    #     self._reset_btn.enabled = False
