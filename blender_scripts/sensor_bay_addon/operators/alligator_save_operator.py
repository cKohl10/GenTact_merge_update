# Description: This script is used to save the sensor positions in the scene to a CSV file.
# The saves are intented for 3D printing with rigged self-capacitance nodes.

# Author: Carson Kohlbrenner
# Date: 6/20/2024

import bpy
import csv
import re
import os
import bpy.props
import numpy as np
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
        self.unit_scale = context.scene.my_addon_properties.unit_scale
        print("AlligatorSaveOperator.execute called\n")
        if self.filepath:  # Check if filepath has been set
            #self.save_attribute_to_csv(context, self.filepath)  # Save the sensor positions to a CSV file
            self.full_save(context)
        else:
            self.report({'WARNING'}, "No file selected")  # Report a warning if no file was selected
            return {'CANCELLED'}
        return {'FINISHED'}
    
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)  # Open file explorer
        return {'RUNNING_MODAL'}
    
    # This function recursively checks the children of the object for sensors
    def check_children_for_sensors(self, obj, parent_path, folder_path, options, write=True):

        sensor_data = {}
        sensor_data_all = []

        # Get the parent path from the root object
        parent_path = parent_path + "/" + obj.name

        is_sensor_attribute_name = "is_sensor"
        pos_attribute_name = "sensor_pos"
        clip_pos_attribute_name = "position"
        stem_pos_attribute_name = "sensor_pos_stem"
        rad_attribute_name = "radii"
        alligator_clip_attribute_name = "is_clip"
        default_radius = False
        use_stem = True

        # Loop through all of the children objects and search for GeometryNodes modifier
        for child in obj.children:
            sensor_data[child.name] = False
            pos_attribute_data = []

            # Recursively check the children for sensors
            child_sensor_data = self.check_children_for_sensors(child, parent_path, folder_path, options, write=write)

            modifier = child.modifiers.get('Skin')

            # Ensure the object has geometry nodes modifier
            if modifier is None:
                # print(f"{child.name} does not have a Skin modifier.")
                # Add the child sensor data to the sensor data list
                sensor_data[child.name] = child_sensor_data
                continue

            #print(f"Found Skin modifier in object {child.name}.")

            # Perform options function
            # Access the node tree within the modifier

            # Set the skin parameters as set by options
            for i in range(len(options.names)):
                modifier[options.names[i]] = options.values[i]

            # Update the scene to reflect changes
            bpy.context.view_layer.update()

            # Get the evaluated geometry
            child.data.update()
            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()
            eval_obj = child.evaluated_get(depsgraph)
            mesh = eval_obj.to_mesh()

            # Update the scene to reflect changes
            bpy.context.view_layer.update()

            # Export the object with skin modifier as an obj file
            self.export_object(child, folder_path)
            
            # Check if the position data exists
            if pos_attribute_name not in mesh.attributes:
                print(f"Attribute {pos_attribute_name} not found in object {child.name}.")
                # for other_name in mesh.attributes:
                #     print(f"Found attribute: {other_name}.")
                # Add the child sensor data to the sensor data list
                sensor_data[child.name] = child_sensor_data
                continue

            if is_sensor_attribute_name not in mesh.attributes:
                print(f"Attribute {is_sensor_attribute_name} not found in object {child.name}.")
                # Add the child sensor data to the sensor data list
                sensor_data[child.name] = child_sensor_data
                continue

            if rad_attribute_name not in mesh.attributes:
                #Set a default radius value if the radii attribute is not found
                print(f"Attribute {rad_attribute_name} not found in object {child.name}. Setting default radius of 0.1.")
                default_radius = True

            if stem_pos_attribute_name not in mesh.attributes:
                #Set a default radius value if the radii attribute is not found
                print(f"Attribute {stem_pos_attribute_name} not found in object {child.name}. Using default sensor positions.")
                use_stem = False
        
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
                        clip_pos.append(SensorData(mesh.attributes[clip_pos_attribute_name].data[i].vector, 0.1, parent_path, True, name=child.name, stem_pos=mesh.attributes[clip_pos_attribute_name].data[i].vector))
            
                # print("\n\n ############################################## \n\n")
                # for clip in clip_pos:
                #     print(f"    Alligator clip position found at {clip.pos} in object {child.name}.")
                # print("\n\n ############################################## \n\n")

            # Add the alligator clip data to the sensor data list as the first element
            # The alligator clips will sandwich the sensor data to complete the node loop
            if len(clip_pos) > 0 and write:
                print(f"Sorting object {child.name} by distance from first clip.")
                sorted_ids = sort(pos_attribute_data, clip_pos[0].pos) # Sort by Distance from first clip
                child_sensor_data.append(clip_pos[0])
            else:
                sorted_ids = range(len(pos_attribute_data)) #Original order
            
            # Add the attribute data to the sensor data list
            sensor_counter = 0
            for i in sorted_ids:
                if is_sensor_data[i].value:
                    if default_radius:
                        child_sensor_data.append(SensorData(pos_attribute_data[i].vector, 0.1, parent_path, name=child.name))
                    else:
                        if use_stem:
                            child_sensor_data.append(SensorData(pos_attribute_data[i].vector, rad_attribute_data[i].value, parent_path, name=child.name, stem_pos=mesh.attributes[stem_pos_attribute_name].data[i].vector))
                        else:
                            child_sensor_data.append(SensorData(pos_attribute_data[i].vector, rad_attribute_data[i].value, parent_path, name=child.name, stem_pos=pos_attribute_data[i].vector))

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

    def hide_children_render(self, obj, value=True):
        for child in obj.children:
            child.hide_render = value
            self.hide_children_render(child)

    # This function saves the sensor positions to a CSV file
    def save_attribute_to_csv(self, context, folder_path, write=True, option=None):
        # Get the object
        obj = context.object

        # Expand the ~ symbol into the path of the home directory
        #file_path = os.path.expanduser(file_path)

        # Create a folder at the folder_path to save all exported files 
        #mesh_folder_path = folder_path + '/meshes'
        os.makedirs(folder_path, exist_ok=True)
        #os.makedirs(mesh_folder_path, exist_ok=True)

        # Make an array of all sensor positions,radii, and parent paths
        sensor_data = []

        if option is None:
            option = Params([],[]) # Default options, do nothing

        # Hide all objects and children from rendering
        obj.hide_render = True
        self.hide_children_render(obj)
        
        # Check the children for sensors
        sensor_data = self.check_children_for_sensors(obj, "", folder_path, option, write=write)

        # Check if there are any sensor positions
        if len(sensor_data) == 0:
            print("No sensor positions found.")
            return

        # Save the attribute data to CSV if write is true
        index_counter = 1
        curr_name = ""
        dir_path = os.path.dirname(folder_path)
        if write:
            with open(dir_path + '/sensor_config.csv', 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Index', 'X', 'Y', 'Z', 'Stem X', 'Stem Y', 'Stem Z','Radius', 'Parent'])
                
                for element in sensor_data:
                    if element.name != curr_name:
                        curr_name = element.name
                        csv_writer.writerow(["", "", "", "", "", "", "", "", ""])
                        csv_writer.writerow([element.name, "", "", "", "", "", "", "", ""])

                    pos = element.pos

                    # Check if the sensor is an alligator clip
                    if element.is_clip:
                        csv_writer.writerow(["CLIP", pos.x * self.unit_scale, pos.y * self.unit_scale, pos.z * self.unit_scale, "", "", "", element.radius, element.parent])
                    else:
                        csv_writer.writerow([index_counter, pos.x * self.unit_scale, pos.y * self.unit_scale, pos.z * self.unit_scale, element.stem_pos.x * self.unit_scale, element.stem_pos.y * self.unit_scale, element.stem_pos.z * self.unit_scale, element.radius, element.parent])
                        index_counter = index_counter + 1
        
        # print(f"\nAttribute {attribute_name} saved to {file_path}")
        print(f"Sensor count: {len(sensor_data)}")

        self.hide_children_render(obj, False)

    # Saves the selected sensors as an STL file
    def export_object(self, ob, file_path):

        mesh_path = file_path + "/" + ob.name + ".stl"
        img_path = file_path + "/" + ob.name + ".png"

        # Ensure the object is in the active view layer
        if ob.name not in bpy.context.view_layer.objects:
            print(f"Object {ob.name} is not in the active view layer.")
            return

        # Ensure the object is not hidden
        if ob.hide_get():
            ob.hide_set(False)

        # Deselect and hide all objects from rendering
        bpy.ops.object.select_all(action='DESELECT')

        # Set the selected object to the active object
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob

        # Make only the sleected object visible to render
        ob.hide_render = False

        # Set the render output path
        bpy.context.scene.render.filepath = img_path
        
        # Render the scene and save as PNG
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.ops.render.render(write_still=True)

        #bpy.ops.export_mesh.stl(filepath=file_path, use_selection=True)
        bpy.ops.wm.stl_export(filepath=mesh_path, export_selected_objects=True, global_scale=self.unit_scale)

        ob.hide_render = True

    def full_save(self, context):
        # Saves three folders, a unified meshes folder, just the sensors folder, and just the skin folder
        folder_path = self.filepath
        os.makedirs(folder_path, exist_ok=True)

        #Jank solution to input socket naming that could potentially be improved
        # ["Show Original Mesh", "Show Skin Mold", "Show Sensors", "Show Alligator Clips", "Unify Meshes"]
        sockets = ["Socket_6", "Socket_24", "Socket_20", "Socket_22", "Socket_39"]

        # Just the skin folder
        skin_folder_path = folder_path + '/Only Skin'
        os.makedirs(skin_folder_path, exist_ok=True)
        skin_params = Params(sockets, [False, True, False, False, False])

        # Just the sensors folder
        sensor_folder_path = folder_path + '/Only Sensors'
        os.makedirs(sensor_folder_path, exist_ok=True)
        sensor_params = Params(sockets, [False, False, True, True, False])

        # Unified meshes folder
        uni_folder_path = folder_path + '/Unified Meshes'
        os.makedirs(uni_folder_path, exist_ok=True)
        uni_params = Params(sockets, [False, True, True, True, True])

        # Full model meshes folder
        full_folder_path = folder_path + '/Full Model'
        os.makedirs(full_folder_path, exist_ok=True)
        full_params = Params(sockets, [True, True, True, False, False])

        self.save_attribute_to_csv(context, skin_folder_path, write=False, option=skin_params)
        self.save_attribute_to_csv(context, sensor_folder_path, write=True, option=sensor_params)
        self.save_attribute_to_csv(context, uni_folder_path, write=False, option=uni_params)
        self.save_attribute_to_csv(context, full_folder_path, write=False, option=full_params)

        # Save copy of blend file to folder
        bpy.ops.wm.save_as_mainfile(filepath=folder_path + '/model.blend')
        
    ############################################################
    ##################### Helper Functions #####################
    ############################################################

def euclidean_distance(point1, point2):
    """Calculate the Euclidean distance between two 3D points."""
    return np.linalg.norm(np.array(point1) - np.array(point2))

def sort(positions, start_pos):
    """Sort the nodes by distance from the start node"""
    remaining_ids = [i for i in range(len(positions))]
    sorted_ids = []

    for i in range(len(positions)):
        dist = float('inf')
        for j in range(len(remaining_ids)):
            if euclidean_distance(positions[remaining_ids[j]].vector, start_pos) < dist:
                dist = euclidean_distance(positions[remaining_ids[j]].vector, start_pos)
                best_id = remaining_ids[j]
        sorted_ids.append(best_id)
        remaining_ids.remove(best_id)

    print(f"Sorted IDs: {sorted_ids}")

    return sorted_ids



# This sensor data class is used to store infomation about each sensor
class SensorData:
    def __init__(self, pos, radius, parent, is_clip=False, name="", stem_pos=None):
        self.pos = pos
        self.radius = radius
        self.parent = parent
        self.is_clip = is_clip
        self.name = name
        self.stem_pos = stem_pos

    def __str__(self):
        return f"Pos: {self.pos}, Radius: {self.radius}, Parent: {self.parent}"

    def __repr__(self):
        return str(self)
    
class Params: # Class to store the parameters for the skin modifier
    def __init__(self, names, values):
        self.names = names
        self.values = values


