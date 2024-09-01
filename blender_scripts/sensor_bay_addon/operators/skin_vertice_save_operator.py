# Description: This script is used to save the sensor positions in the scene to a CSV file.
# The saves are intented for import in the Isaac Sim "Contact Extension" plugin.

# Author: Carson Kohlbrenner
# Date: 6/20/2024

import bpy
import csv
import re
import bpy.props
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty
from bpy.types import Operator

############################################################

class SkinVerticeSaveOperator(Operator, ExportHelper):
    """Saves the sensors in the scene"""
    bl_idname = "object.skin_vertice_save_operator"
    bl_label = "Save Skin Vertices"

    filename_ext = ".csv"
    filter_glob: StringProperty(
        default="*.csv",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )
    
    def execute(self, context):
        print("SkinVerticeSaveOperator.execute called\n")
        self.skin_group_name = context.scene.my_addon_properties.group_name
        if self.filepath:  # Check if filepath has been set
            self.save_attribute_to_csv(context, self.filepath)
        else:
            self.report({'WARNING'}, "No file selected")  # Report a warning if no file was selected
            return {'CANCELLED'}
        return {'FINISHED'}
    
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)  # Open file explorer
        return {'RUNNING_MODAL'}
    
############################################################
##################### Helper Functions #####################
############################################################

    class VertexData:
        def __init__(self, index, pos, obj_name):
            self.index = index
            self.pos = pos
            self.obj_name = obj_name

        def __str__(self):
            return f"Index: {self.index}, Pos: {self.pos}"

        def __repr__(self):
            return str(self)

    def check_children_for_skin(self, obj, parent_path):

        vertice_data = {}
        vertice_data_all = []

        # Get the parent path from the root object
        parent_path = parent_path + "/" + obj.name

        pos_attribute_name = "position"

        # Loop through all of the children objects and search for GeometryNodes modifier
        for child in obj.children:
            vertice_data[child.name] = False
            pos_attribute_data = []

            # Recursively check the children for sensors
            child_vertice_data = self.check_children_for_skin(child, parent_path)

            # Ensure the object has geometry nodes modifier
            if child.modifiers.get('Skin') is None:
                #print(f"{child.name} does not have a Skin modifier.")
                # Add the child sensor data to the sensor data list
                vertice_data[child.name] = child_vertice_data
                continue

            #print(f"Found Skin modifier in object {child.name}.")

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
                vertice_data[child.name] = child_vertice_data
                continue

            # Get world position of child object's origin
            #origin_pos = child.matrix_world.translation

            pos_attribute_data = mesh.attributes[pos_attribute_name].data

            # Get path to object
            path = parent_path + "/" + child.name
            # Remove any triple digit numbers from the parent path
            #parent_path = re.sub(r'\.\d{3,}', '', parent_path)

            vg_index = child.vertex_groups[self.skin_group_name].index

            vgVerts = [ v for v in child.data.vertices if vg_index in [ vg.group for vg in v.groups ] ]

            #print(f"Found {sensor_counter} sensor positions in object {child.name} under {parent_path}.")
            for vert in vgVerts:
                pos = pos_attribute_data[vert.index].vector
                child_vertice_data.append(self.VertexData(vert.index, pos, child.name))

                #print(f"Vertex {vert.index} at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) in object {child.name} under {path}.")

            # Clean up
            #eval_obj.to_mesh_clear()

            # Add the child sensor data to the sensor data list
            vertice_data[child.name] = child_vertice_data

        # Print the sensor data
        if len(obj.children) == 0:
            return vertice_data_all
        else:
            print(f"\nObject {obj.name} has {len(obj.children)} child(ren):")
            for child in obj.children:
                if vertice_data[child.name] == False:
                    print(f"        Child: {child.name} has no skin vertices.")
                else:
                    print(f"        Child: {child.name} has {len(vertice_data[child.name])} vertices.")

                    for vertex in vertice_data[child.name]:
                        vertice_data_all.append(vertex)

            print(f"Returning {len(vertice_data_all)} vertice positions in object {obj.name}")
        return vertice_data_all


    def save_attribute_to_csv(self, context, file_path):
        # Get the object
        obj = context.object

        # Expand the ~ symbol into the path of the home directory
        #file_path = os.path.expanduser(file_path)

        # Make an array of all sensor positions,radii, and parent paths
        vertice_data = []
        
        # Check the children for sensors
        vertice_data = self.check_children_for_skin(obj, "")

        # Check if there are any sensor positions
        if len(vertice_data) == 0:
            print("No vertex positions found.")
            return

        # Save the attribute data to CSV
        with open(file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Index', 'X', 'Y', 'Z', 'Object Name'])
            
            for i, element in enumerate(vertice_data):
                pos = element.pos
                csv_writer.writerow([element.index, pos.x, pos.y, pos.z, element.obj_name])
        
        # print(f"\nAttribute {attribute_name} saved to {file_path}")
        print(f"Vertice count: {len(vertice_data)}")

