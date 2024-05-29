# Save Sensor Position Script
# Author: Carson Kohlbrenner
# Date Created: 5/29/2024
# Last Modified: 5/29/2024

# Description: This exports all sensor locations in Blender into a CSV file that Isaac Sim can accept.
#
# Output CSV format:
# SensorNum, X, Y, Z, radius, parent_path

import bpy
import bmesh

# Get the active object
obj = bpy.context.active_object

# Ensure we are in the correct mode
bpy.ops.object.mode_set(mode='OBJECT')

# Get the geometry data
mesh = obj.data

# Create a BMesh to access vertex data
bm = bmesh.new()
bm.from_mesh(mesh)

# Get the custom attribute (assuming it is a point cloud)
positions = [v.co for v in bm.verts]

# Write positions to a file
output_file = "C:/path_to_your_file/instance_positions.txt"

with open(output_file, 'w') as f:
    for pos in positions:
        f.write(f"{pos.x}, {pos.y}, {pos.z}\n")

bm.free()
print(f"Instance positions written to {output_file}")
