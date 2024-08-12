# Description: This script is used to save the sensor positions in the scene to a CSV file.
# The saves are intented for 3D printing with rigged self-capacitance nodes.

# Author: Carson Kohlbrenner
# Date: 6/20/2024

import bpy
import csv
import re
import os
import bpy.props
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty
from bpy.types import Operator

############################################################

class AlligatorSaveOperator(Operator, ExportHelper):
    """Saves the sensors in the scene"""
    bl_idname = "object.alligator_save_operator"
    bl_label = "Alligator Save"

    filename_ext = ""
    filter_glob: StringProperty(
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )
    
    def execute(self, context):
        print("AlligatorSaveOperator.execute called\n")
        if self.filepath:  # Check if filepath has been set
            save_attribute_to_csv(context, self.filepath)
        else:
            self.report({'WARNING'}, "No file selected")  # Report a warning if no file was selected
            return {'CANCELLED'}
        return {'FINISHED'}
    
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)  # Open file explorer
        return {'RUNNING_MODAL'}
    
    # This function recursively checks the children of the object for sensors
def check_children_for_sensors(obj, parent_path, folder_path):

    sensor_data = {}
    sensor_data_all = []

    # Get the parent path from the root object
    parent_path = parent_path + "/" + obj.name

    is_sensor_attribute_name = "is_sensor"
    pos_attribute_name = "position"
    rad_attribute_name = "radii"
    alligator_clip_attribute_name = "is_clip"
    default_radius = False

    # Loop through all of the children objects and search for GeometryNodes modifier
    for child in obj.children:
        sensor_data[child.name] = False
        pos_attribute_data = []

        # Recursively check the children for sensors
        child_sensor_data = check_children_for_sensors(child, parent_path, folder_path)

        # Ensure the object has geometry nodes modifier
        if child.modifiers.get('Skin') is None:
            # print(f"{child.name} does not have a Skin modifier.")
            # Add the child sensor data to the sensor data list
            sensor_data[child.name] = child_sensor_data
            continue

        #print(f"Found Skin modifier in object {child.name}.")

        # Export the object with skin modifier as an obj file
        export_object(child, folder_path + "/" + child.name + ".obj")

        # Get the evaluated geometry
        depsgraph = bpy.context.evaluated_depsgraph_get()
        eval_obj = child.evaluated_get(depsgraph)
        mesh = eval_obj.to_mesh()
        
        # Check if the position data exists
        if pos_attribute_name not in mesh.attributes:
            #print(f"Attribute {attribute_name} not found in object {child.name}.")
            # for other_name in mesh.attributes:
            #     print(f"Found attribute: {other_name}.")
            # Add the child sensor data to the sensor data list
            sensor_data[child.name] = child_sensor_data
            continue

        if is_sensor_attribute_name not in mesh.attributes:
            #print(f"Attribute {is_sensor_attribtue_name} not found in object {child.name}.")
            # Add the child sensor data to the sensor data list
            sensor_data[child.name] = child_sensor_data
            continue

        if rad_attribute_name not in mesh.attributes:
            #Set a default radius value if the radii attribute is not found
            print(f"Attribute {rad_attribute_name} not found in object {child.name}. Setting default radius of 0.1.")
            default_radius = True
    
        # Get the attribute data

        pos_attribute_data = mesh.attributes[pos_attribute_name].data

        # Get the radii attribute data
        rad_attribute_data = []
        if not default_radius:
            rad_attribute_data = mesh.attributes[rad_attribute_name].data

        is_sensor_data = mesh.attributes[is_sensor_attribute_name].data

        # Get path to object
        parent_path = parent_path + "/" + child.name
        # Remove any triple digit numbers from the parent path
        parent_path = re.sub(r'\.\d{3,}', '', parent_path)

        # Check if the alligator clip attribute exists
        clip_pos = []
        if alligator_clip_attribute_name in mesh.attributes:
            is_clip_data = mesh.attributes[alligator_clip_attribute_name].data
            # Get the position data of the alligator clip
            for i in range(len(is_clip_data)):
                if is_clip_data[i].value:
                    clip_pos.append(SensorData(mesh.attributes[pos_attribute_name].data[i].vector, 0.1, parent_path, True))
        
            print("\n\n ############################################## \n\n")
            for clip in clip_pos:
                print(f"    Alligator clip position found at {clip.pos} in object {child.name}.")
            print("\n\n ############################################## \n\n")

        # Add the alligator clip data to the sensor data list as the first element
        # The alligator clips will sandwich the sensor data to complete the node loop
        if len(clip_pos) > 0:
            child_sensor_data.append(clip_pos[0])
        
        # Add the attribute data to the sensor data list
        sensor_counter = 0
        for i in range(len(pos_attribute_data)):
            if is_sensor_data[i].value:
                if default_radius:
                    child_sensor_data.append(SensorData(child.name, pos_attribute_data[i].vector, 0.1, parent_path))
                else:
                    child_sensor_data.append(SensorData(child.name, pos_attribute_data[i].vector, rad_attribute_data[i].value, parent_path))

                sensor_counter = sensor_counter + 1

        #print(f"Found {sensor_counter} sensor positions in object {child.name} under {parent_path}.")

        # Clean up
        #eval_obj.to_mesh_clear()

        # Add the second alligator clip data to the sensor data list as the last element is it exists
        if len(clip_pos) > 1:
            child_sensor_data.append(clip_pos[1])

        # Add the child sensor data to the sensor data list
        sensor_data[child.name] = child_sensor_data

    # Print the sensor data
    if len(obj.children) == 0:
        return sensor_data_all
    else:
        total_sensor_count = 0
        print(f"\nObject {obj.name} has {len(obj.children)} child(ren):")
        for child in obj.children: # Loop through all of the children objects
            # Identify how many of the positions in the child are sensors and not clips
            num_sensors = 0
            for sensor in sensor_data[child.name]:
                if sensor.is_clip == False:
                    num_sensors = num_sensors + 1

            if num_sensors == 0:
                print(f"        Child: {child.name} has no sensors.")
            else:
                print(f"        Child: {child.name} has {num_sensors} sensors.")
                total_sensor_count = total_sensor_count + num_sensors

                for sensor in sensor_data[child.name]:
                    sensor_data_all.append(sensor)

        print(f"Returning {total_sensor_count} sensor positions in object {obj.name} under {parent_path}.")
    return sensor_data_all

# This function saves the sensor positions to a CSV file
def save_attribute_to_csv(context, folder_path):
    # Get the object
    obj = context.object

    # Expand the ~ symbol into the path of the home directory
    #file_path = os.path.expanduser(file_path)

    # Create a folder at the folder_path to save all exported files 
    mesh_folder_path = folder_path + '/meshes'
    os.makedirs(folder_path, exist_ok=True)
    os.makedirs(mesh_folder_path, exist_ok=True)

    # Make an array of all sensor positions,radii, and parent paths
    sensor_data = []
    
    # Check the children for sensors
    sensor_data = check_children_for_sensors(obj, "", mesh_folder_path)

    # Check if there are any sensor positions
    if len(sensor_data) == 0:
        print("No sensor positions found.")
        return

    # Save the attribute data to CSV
    index_counter = 1
    with open(folder_path + '/sensor_config.csv', 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['Index', 'X', 'Y', 'Z', 'Radius', 'Parent'])
        
        for element in sensor_data:
            if element.name != curr_name:
                curr_name = element.name
                csv_writer.writerow(["element.name"])
                csv_writer.writerow([""])

            pos = element.pos

            # Check if the sensor is an alligator clip
            if element.is_clip:
                csv_writer.writerow(["CLIP", pos.x, pos.y, pos.z, element.radius, element.parent])
            else:
                csv_writer.writerow([index_counter, pos.x, pos.y, pos.z, element.radius, element.parent])
                index_counter = index_counter + 1
    
    # print(f"\nAttribute {attribute_name} saved to {file_path}")
    print(f"Sensor count: {len(sensor_data)}")

# Saves the selected sensors as an STL file
def export_object(ob, file_path):

    # Deselect all objects
    bpy.ops.object.select_all(action='DESELECT')

    # Set the selected object to the active object
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob

    #bpy.ops.export_mesh.stl(filepath=file_path, use_selection=True)
    bpy.ops.wm.stl_export(filepath=file_path, export_selected_objects=True, use_scene_unit=True)
    
############################################################
##################### Helper Functions #####################
############################################################

# This sensor data class is used to store infomation about each sensor
class SensorData:
    def __init__(self, name, pos, radius, parent, is_clip=False):
        self.pos = pos
        self.radius = radius
        self.parent = parent
        self.is_clip = is_clip
        self.name = name

    def __str__(self):
        return f"Pos: {self.pos}, Radius: {self.radius}, Parent: {self.parent}"

    def __repr__(self):
        return str(self)


