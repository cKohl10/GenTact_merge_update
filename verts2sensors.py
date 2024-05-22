import csv
import re
from pxr import Usd, UsdGeom

def remove_repeated_prims(path):
    parts = path.split('/')
    for i in range(len(parts) - 1, 0, -1):
        if parts[i] == parts[i - 1]:
            del parts[i]
    return '/'.join(parts)

def extract_vertices(usd_file, csv_file):
    # Open the USD file
    stage = Usd.Stage.Open(usd_file)
    
    # Open the CSV file for writing
    with open(csv_file, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write the header row
        csvwriter.writerow(['SensorNum', 'X', 'Y', 'Z', 'parent_path'])

        vert_count = 1
        
        # Iterate through all the prims in the stage
        for prim in stage.Traverse():
            # Check if the prim is a geometry (has points)
            if prim.IsA(UsdGeom.PointBased):
                geom = UsdGeom.PointBased(prim)
                # Get the points (vertices) of the geometry
                points = geom.GetPointsAttr().Get()
                parent_path = prim.GetPath().pathString

                # Omit 'Root' from the parent path
                parent_path = parent_path.replace('/Root', '')

                # Remove any triple digit numbers from the parent path
                parent_path = re.sub(r'_\d{3,}', '', parent_path)

                # Blender splits the vertices into seperate objects, so we need to remove the repeated prims
                parent_path = remove_repeated_prims(parent_path)
                
                # Write each point to the CSV
                for point in points:
                    csvwriter.writerow([vert_count, point[0], point[1], point[2], 0.1, parent_path])
                    vert_count += 1

    print(f'Vertices extracted from {usd_file} and saved to {csv_file}')
    print(f'Vertex count: {vert_count - 1}')
                    
# Example usage
usd_file = 'scenes/Ant Scene/robots/Ant_decimated.usd'
csv_file = 'sensor_configs/ant_vertices.csv'
extract_vertices(usd_file, csv_file)
