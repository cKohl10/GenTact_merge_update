bl_info = {
    "name": "Tactile Sensor Bay",
    "author": "Carson Kohlbrenner",
    "version": (1, 0),
    "blender": (2, 80, 0),
    "location": "Object > Sensor Bay",
    "description": "Paint on tactile sensors over a surface and save them for Isaac Sim",
    "warning": "",
    "doc_url": "https://github.com/cKohl10/TactileSim",
    "category": "Object",
}

import bpy
import bmesh
import csv
import os
import re
import bpy.props
from bpy.utils import resource_path
from pathlib import Path
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty
from bpy.types import Operator

class SensorData:
    def __init__(self, pos, radius, parent):
        self.pos = pos
        self.radius = radius
        self.parent = parent

    def __str__(self):
        return f"Pos: {self.pos}, Radius: {self.radius}, Parent: {self.parent}"

    def __repr__(self):
        return str(self)

def check_children_for_sensors(obj, parent_path):

    sensor_data = {}
    sensor_data_all = []

    # Get the parent path from the root object
    parent_path = parent_path + "/" + obj.name

    is_sensor_attribute_name = "is_sensor"
    pos_attribute_name = "sensor_pos"
    rad_attribute_name = "radii"
    default_radius = False

    # Loop through all of the children objects and search for GeometryNodes modifier
    for child in obj.children:
        sensor_data[child.name] = False
        pos_attribute_data = []

        # Recursively check the children for sensors
        child_sensor_data = check_children_for_sensors(child, parent_path)

        # Ensure the object has geometry nodes modifier
        if child.modifiers.get('Skin') is None:
            #print(f"{child.name} does not have a Skin modifier.")
            # Add the child sensor data to the sensor data list
            sensor_data[child.name] = child_sensor_data
            continue

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

        # Add the attribute data to the sensor data list
        sensor_counter = 0
        for i in range(len(pos_attribute_data)):
            if is_sensor_data[i].value:
                if default_radius:
                    child_sensor_data.append(SensorData(pos_attribute_data[i].vector, 0.1, parent_path))
                else:
                    child_sensor_data.append(SensorData(pos_attribute_data[i].vector, rad_attribute_data[i].value, parent_path))

                sensor_counter = sensor_counter + 1

        #print(f"Found {sensor_counter} sensor positions in object {child.name} under {parent_path}.")

        # Clean up
        #eval_obj.to_mesh_clear()

        # Add the child sensor data to the sensor data list
        sensor_data[child.name] = child_sensor_data

    # Print the sensor data
    if len(obj.children) == 0:
        return sensor_data_all
    else:
        print(f"\nObject {obj.name} has {len(obj.children)} child(ren):")
        for child in obj.children:
            if sensor_data[child.name] == False:
                print(f"        Child: {child.name} has no sensors.")
            else:
                print(f"        Child: {child.name} has {len(sensor_data[child.name])} sensors.")

                for sensor in sensor_data[child.name]:
                    sensor_data_all.append(sensor)

        print(f"Returning {len(sensor_data_all)} sensor positions in object {obj.name} under {parent_path}.")
    return sensor_data_all


def save_attribute_to_csv(context, file_path):
    # Get the object
    obj = context.object

    # Expand the ~ symbol into the path of the home directory
    #file_path = os.path.expanduser(file_path)

    # Make an array of all sensor positions,radii, and parent paths
    sensor_data = []
    
    # Check the children for sensors
    sensor_data = check_children_for_sensors(obj, "")

    # Check if there are any sensor positions
    if len(sensor_data) == 0:
        print("No sensor positions found.")
        return

    # Save the attribute data to CSV
    with open(file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['Index', 'X', 'Y', 'Z', 'Radius', 'Parent'])
        
        for i, element in enumerate(sensor_data):
            pos = element.pos
            csv_writer.writerow([i, pos.x, pos.y, pos.z, element.radius, element.parent])
    
    # print(f"\nAttribute {attribute_name} saved to {file_path}")
    print(f"Sensor count: {len(sensor_data)}")

class SensorSaveOperator(Operator, ExportHelper):
    """Saves the sensors in the scene"""
    bl_idname = "object.save_sensors_operator"
    bl_label = "Save Sensor Positions"

    filename_ext = ".csv"
    filter_glob: StringProperty(
        default="*.csv",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )
    
    def execute(self, context):
        print("SensorSaveOperator.execute called\n")
        if self.filepath:  # Check if filepath has been set
            save_attribute_to_csv(context, self.filepath)
        else:
            self.report({'WARNING'}, "No file selected")  # Report a warning if no file was selected
            return {'CANCELLED'}
        return {'FINISHED'}
    
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)  # Open file explorer
        return {'RUNNING_MODAL'}
    
class ApplySkinOperator(bpy.types.Operator):
    bl_idname = "object.apply_skin"
    bl_label = "Apply Skin"
    bl_options = {'REGISTER', 'UNDO'}

    # Add a property to hold the geometry node name
    geo_node_name: bpy.props.StringProperty(name="Geometry Node Name", default="Skin")

    def execute(self, context):
        print("ApplySkinOperator.execute called\n")

        # Find the path to the geometry node as a zipped addon
        USER = Path(resource_path('USER'))
        ADDON = "sensor_bay_addon"
        srcPath = USER / "scripts/addons" / ADDON / "Assets" / "gemoetry_nodes.blend"
        srcFile = str(srcPath)
        
        # Path to the .blend file containing the geometry node group
        geo_node_path = srcFile

        # Load the geometry node group from the .blend file
        #  # Path to the add-on's directory
        # addon_path = os.path.dirname(__file__)
        
        # # Path to the .blend file containing the geometry node group
        # geo_node_path = os.path.join(addon_path, "geometry_nodes.blend")
        
        # Load the geometry node group from the .blend file
        with bpy.data.libraries.load(geo_node_path) as (data_from, data_to):
            if self.geo_node_name in data_from.node_groups:
                data_to.node_groups = [self.geo_node_name]
        
        # Get the geometry node group
        geo_node_group = bpy.data.node_groups.get(self.geo_node_name)
        
        if geo_node_group is None:
            self.report({'ERROR'}, self.geo_node_name + " not found")
            return {'CANCELLED'}
        
        # Apply the geometry node group to the selected objects
        for obj in context.selected_objects:
            if obj.type == 'MESH':
                mod = obj.modifiers.new(name="GeometryNodes", type='NODES')
                mod.node_group = geo_node_group

        return {'FINISHED'}


class SensorPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "Sensor Bay"
    bl_idname = "OBJECT_PT_SENSOR_BAY"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    def draw(self, context):
        layout = self.layout

        obj = context.object

        row = layout.row()
        row.label(text="Save sensors as CSV", icon='FILE_TICK')

        row = layout.row()
        row.label(text="Selected root prim: " + obj.name)
        row = layout.row()
        row.prop(obj, "name")

        # Save Button 
        row = layout.row()
        row.operator("object.save_sensors_operator")

        # Apply Skin Button
        row = layout.row()
        row.operator("object.apply_skin", text="Apply Skin").geo_node_name = "Skin"


def register():
    bpy.utils.register_class(SensorSaveOperator)
    bpy.utils.register_class(ApplySkinOperator)
    bpy.utils.register_class(SensorPanel)

def unregister():
    bpy.utils.unregister_class(SensorSaveOperator)
    bpy.utils.unregister_class(ApplySkinOperator)
    bpy.utils.unregister_class(SensorPanel)
    

if __name__ == "__main__":
    register()
