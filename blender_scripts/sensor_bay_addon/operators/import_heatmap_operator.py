# Description: Imports a series of heatmaps from a CSV file by assigning the respective weights to vertices of each object
# Requires that the geometry of the robot is not altered between the export and import of the heatmaps

# Author: Carson Kohlbrenner
# Date: 6/20/2024

import bpy
import csv
import bpy.props
import numpy as np
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty
from bpy.types import Operator

class ImportHeatmapOperator(Operator, ExportHelper):
    """Imports a heatmap given by vertex indices and weights"""
    bl_idname = "object.import_heatmap_operator"
    bl_label = "Import Heatmap"

    filepath = bpy.props.StringProperty(subtype="FILE_PATH")
    filename_ext = ".csv"
    filter_glob: StringProperty(
        default="*.csv",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    # This dictionary will store an array of VertexData objects for each object in the scene to be changed
    vertex_dict = {}

    class VertexData:
        def __init__(self, index, weight, obj_name, pos=None):
            self.index = index
            self.weight = weight
            self.obj_name = obj_name
            self.pos = pos

        def __str__(self):
            return f"Index: {self.index}, Weight: {self.weight}"

        def __repr__(self):
            return str(self)

    def execute(self, context):
        print("ImportHeatmapOperator.execute called\n")
        self.vx_group_name = context.scene.my_addon_properties.group_name
        if self.filepath:  # Check if filepath has been set
            self.apply_heatmap(context)
        else:
            self.report({'WARNING'}, "No file selected")  # Report a warning if no file was selected
            return {'CANCELLED'}
        return {'FINISHED'}
    
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)  # Open file explorer
        return {'RUNNING_MODAL'}
    
    def apply_heatmap(self, context):
        # Get the object
        obj = context.object

        # Import the heatmap data
        self.import_csv()

        # Begin applying the heatmap
        self.traverse_children(obj, "")

    def traverse_children(self, obj, parent_path):
        for child in obj.children:
            self.traverse_children(child, parent_path)

            # Check if the child is not in vertex_dict
            if child.name not in self.vertex_dict:
                #print(f"Could not find {child.name} in vertex_dict")
                continue

            # Check if the child has a node placement vertex group
            if not child.vertex_groups.get(self.vx_group_name):
                print(f"Vertex group {self.vx_group_name} not found in {child.name}, creating it")
                
                # If not, create the vertex group
                child.vertex_groups.new(name=self.vx_group_name)

            # Apply the weights to the vertices
            vertex_set = self.vertex_dict[child.name]
            vxg = child.vertex_groups.get(self.vx_group_name)
            change_count = 0
            for v in vertex_set:
                try:
                    vxg.add([v.index], v.weight, 'REPLACE')
                    change_count += 1
                except:
                    print(f"Missing vertex! {v.index} not found in {child.name}! Has the geometry changed?")

            print(f"Modified {change_count} vertice weights in {child.name}")

    def import_csv(self):

        """Expecting CSV file format to be: [index, weight, object_name]"""

        # Open the CSV file
        data = np.genfromtxt(self.filepath, delimiter=',', skip_header=1, dtype=str)

        # Group all the vertices with the same object name together in an array of VertexData objects
        for row in data:
            index = int(row[0])
            weight = float(row[1])
            obj_name = row[2]

            # Check if the object name is in the dictionary
            if obj_name not in self.vertex_dict:
                self.vertex_dict[obj_name] = []
                print(f"Creating new object {obj_name} in vertex_dict")

            # Add the VertexData object to the dictionary
            self.vertex_dict[obj_name].append(self.VertexData(index, weight, obj_name))
            
