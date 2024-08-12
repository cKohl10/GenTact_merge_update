bl_info = {
    "name": "Tactile Sensor Bay",
    "author": "Carson Kohlbrenner",
    "version": (1, 6),
    "blender": (2, 80, 0),
    "location": "Object > Sensor Bay",
    "description": "Paint on tactile sensors over a surface and save them for Isaac Sim",
    "warning": "",
    "doc_url": "https://github.com/cKohl10/TactileSim",
    "category": "Object",
}

import bpy
import bpy.props

from .operators.isaac_save_operator import IsaacSaveOperator
from .operators.alligator_save_operator import AlligatorSaveOperator

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

        # Isaac Save Button 
        row = layout.row()
        row.operator("object.isaac_save_operator")

        # Alligator Save Button 
        row = layout.row()
        row.operator("object.alligator_save_operator")

        # Apply Skin Button
        # row = layout.row()
        # row.operator("object.apply_skin", text="Apply Skin").geo_node_name = "Skin"


def register():
    bpy.utils.register_class(IsaacSaveOperator)
    bpy.utils.register_class(AlligatorSaveOperator)
    bpy.utils.register_class(SensorPanel)

def unregister():
    bpy.utils.unregister_class(IsaacSaveOperator)
    bpy.utils.unregister_class(AlligatorSaveOperator)
    bpy.utils.unregister_class(SensorPanel)
    

if __name__ == "__main__":
    register()
