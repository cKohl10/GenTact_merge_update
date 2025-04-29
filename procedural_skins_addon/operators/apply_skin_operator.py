import bpy
import bpy.props
from bpy.utils import resource_path
from pathlib import Path

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
