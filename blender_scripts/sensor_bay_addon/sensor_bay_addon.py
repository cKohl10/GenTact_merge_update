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

def save_sensors(context):
    pass

class SensorSaveOperator(bpy.types.Operator):
    """Saves the sensors in the scene"""
    bl_idname = "object.save_sensors_operator"
    bl_label = "Save Sensor Positions"
    
    def execute(self, context):
        save_sensors(context)
        return {'FINISHED'}

class SensorPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "Sensor Bay"
    bl_idname = "SENSOR_BAY"
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
