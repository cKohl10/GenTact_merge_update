# This software contains source code provided by NVIDIA Corporation.
# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from .AbstracSensorClass import AbstractSensorOperator
from .ContactSensorClass import ContactSensorOperator

import numpy as np
import omni.timeline
import omni.ui as ui
import omni.kit.commands
import time
import os
import sys
import carb
from omni.isaac.core.utils.prims import is_prim_path_valid, get_all_matching_child_prims, delete_prim, get_prim_children
from omni.isaac.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
from omni.isaac.core.world import World
from omni.isaac.ui.element_wrappers import CollapsableFrame, StateButton, Button, TextBlock, StringField, DropDown
from omni.isaac.ui.ui_utils import get_style, LABEL_WIDTH
from omni.usd import StageEventType
from pxr import Sdf, UsdLux, Gf
from omni.isaac.sensor import _sensor
#from omni.isaac.proximity_sensor import Sensor, register_sensor, clear_sensors

from .scenario import ExampleScenario
from pxr import UsdPhysics


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

        # Create a list to hold all sensor operators
        self._sensor_operators = []

        ############### Add Sensor Operators Here ################
        self._sensor_operators.append(ContactSensorOperator()) # Add a contact sensor operator
        #########################################################
    

        # Debugging
        # version = sys.version
        # executable = sys.executable
        # print(f"Python version: {version}")
        # print(f"Python executable location: {executable}")


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
        
        # Update the sensor readings for all added sensors
        for operator in self._sensor_operators:
            operator.sensor_update(step)

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

        #link the status report frame to the sensor operators to allow them to update the status report
        for operator in self._sensor_operators:
            operator._status_report_field = self._status_report_field

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

                # Add a button to all supported sensor operators
                for operator in self._sensor_operators:
                    button = Button(
                        "Refresh " + operator.sensor_description,
                        "Update",
                        tooltip="Reread the data from the specified file path to update sensors",
                        on_click_fn=operator.import_sensors_fn,
                    )
                    self.wrapped_ui_elements.append(button)

                self._status_report_field = TextBlock(
                    "Import Status",
                    num_lines=10,
                    tooltip="Outputs the status of the import process",
                    include_copy_button=True,
                )

                # Add the status report field to the sensor operators
                for operator in self._sensor_operators:
                    operator._status_report_field = self._status_report_field

    def create_all_sensor_readings_frames(self):
        # Create a sensor readings frame for each sensor operator
        for operator in self._sensor_operators:
            operator.create_sensor_readings_frame()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """

        self.create_import_sensors_frame()
        #self.create_all_sensor_readings_frames()

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

            # Add an empty string to the beginning of the list
            options.insert(0, "")

            # Add a 'Go Back' option at the end of the list
            options.append("Go Back")
        except:
            options = []

        return options

    def _on_string_field_value_changed_fn(self, value):
        """
        Function that executes when the user changes the value of the string field
        Sets the value of the string field to what the user entered
        """
        self.wrapped_ui_elements[0].set_value(value)

        # Update the dropdown with the new path
        self.wrapped_ui_elements[1].repopulate()

    def _on_dropdown_item_selection(self, item):
        """
        Function that executes when the user selects an item from the dropdown
        Sets the value of the dropdown to the item the user selected
        """

        # Go back if the user selects the empty string
        if item == "Go Back":
            # Get the path from the string field minus the last folder
            path = self.wrapped_ui_elements[0].get_value()
            parts = path.split("/")
            path = "/".join(parts[:-1])
            self.wrapped_ui_elements[0].set_value(path)
            self.wrapped_ui_elements[1].repopulate()
            return
        
        if item == "":
            return 
        
        self.config_path = self.wrapped_ui_elements[0].get_value() + "/" + item

        # Update the config path in the sensor operators
        for operator in self._sensor_operators:
            operator.config_path = self.config_path

        if self.config_path[-4:] != ".csv":
            # If the user selects a file that is not a CSV file, and it is a folder, update the string field with the new path
            self.wrapped_ui_elements[0].set_value(self.config_path)
            self.wrapped_ui_elements[1].repopulate()
        pass


    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def countdown(self, seconds, name=""):
        for i in range(seconds, 0, -1):
            #message += str(i) + "...\n"
            # self._status_report_field.set_text(message)
            print(name + " Countdown: " + str(i))
            time.sleep(1)
        print("\n")

        return

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

    def _reset_extension(self):
        """This is called when the user opens a new stage from self.on_stage_event().
        All state should be reset.
        """
        self._on_init()
        self._reset_ui()

    # def _reset_ui(self):
    #     self._scenario_state_btn.reset()
    #     self._scenario_state_btn.enabled = False
    #     self._reset_btn.enabled = False
