bl_info = {
    "name": "Tactile Sensor Bay",
    "author": "Carson Kohlbrenner",
    "version": (1, 0),
    "blender": (2, 80, 0),
    "location": "View3D > Add > Mesh > New Object",
    "description": "Paint on tactile sensors over a surface and save them for Isaac Sim",
    "warning": "",
    "doc_url": "https://github.com/cKohl10/TactileSim",
    "category": "",
}

import bpy
import bmesh
import csv
import os
import re
from bpy.utils import resource_path
from pathlib import Path

class SensorData:
    def __init__(self, pos, radius, parent):
        self.pos = pos
        self.radius = radius
        self.parent = parent

    def __str__(self):
        return f"Pos: {self.pos}, Radius: {self.radius}, Parent: {self.parent}"

    def __repr__(self):
        return str(self)

def check_children_for_sensors(obj, attribute_name, sensor_data, parent_path):

    # Get the parent path from the root object
    parent_path = parent_path + "/" + obj.name

    # Loop through all of the children objects and search for GeometryNodes modifier
    for child in obj.children:

        # Recursively check the children for sensors
        check_children_for_sensors(child, attribute_name, sensor_data, parent_path)

        # Loop through all of the children objects and search for GeometryNodes modifier
        #print(f"\nChecking object {child.name} under {parent_path}...")

        # Ensure the object has geometry nodes modifier
        if child.modifiers.get('Skin') is None:
            #print(f"{child.name} does not have a Skin modifier.")
            continue

        # Get the evaluated geometry
        depsgraph = bpy.context.evaluated_depsgraph_get()
        eval_obj = child.evaluated_get(depsgraph)
        mesh = eval_obj.to_mesh()
        
        # Check if the attribute exists
        if attribute_name not in mesh.attributes:
            #print(f"Attribute {attribute_name} not found in object {child.name}.")
            # for other_name in mesh.attributes:
            #     print(f"Found attribute: {other_name}.")
            continue
        
        # Get the attribute data
        attribute_data = mesh.attributes[attribute_name].data
        # Remove all zero vectors
        attribute_data = [element for element in attribute_data if element.vector.length > 0]

        # Get path to object
        parent_path = parent_path + "/" + child.name
        # Remove any triple digit numbers from the parent path
        parent_path = re.sub(r'(?<=\.)\d{3,}', '', parent_path)

        # Add the attribute data to the sensor data list
        for element in attribute_data:
            sensor_data.append(SensorData(element.vector, 0.1, parent_path))

        print(f"Found {len(attribute_data)} sensor positions in object {child.name}.")

        # Clean up
        eval_obj.to_mesh_clear()


def save_attribute_to_csv(context):
    # Get the object
    obj = context.object
    attribute_name = "sensor_pos"
    file_path = "~/TactileSim/sensor_configs/UR10/second_model.csv"  # Modify this to a valid path

    # Expand the ~ symbol into the path of the home directory
    file_path = os.path.expanduser(file_path)

    # Make an array of all sensor positions,radii, and parent paths
    sensor_data = []
    
    # Check the children for sensors
    check_children_for_sensors(obj, attribute_name, sensor_data, "")

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
    
    print(f"\nAttribute {attribute_name} saved to {file_path}")
    print(f"Sensor count: {len(sensor_data)}")

class SensorSaveOperator(bpy.types.Operator):
    """Saves the sensors in the scene"""
    bl_idname = "object.save_sensors_operator"
    bl_label = "Save Sensor Positions"
    
    def execute(self, context):
        print("SensorSaveOperator.execute called\n")
        save_attribute_to_csv(context)
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

        row = layout.row()
        row.operator("object.save_sensors_operator")


def register():
    bpy.utils.register_class(SensorSaveOperator)
    bpy.utils.register_class(SensorPanel)

def unregister():
    bpy.utils.unregister_class(SensorSaveOperator)
    bpy.utils.unregister_class(SensorPanel)
    

if __name__ == "__main__":
    register()
