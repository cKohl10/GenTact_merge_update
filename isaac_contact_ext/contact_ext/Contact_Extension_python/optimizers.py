# Define heuristics for the optimizer here

import numpy as np
from pxr import Gf
import omni.ui as ui
from omni.isaac.ui.element_wrappers import CollapsableFrame
from omni.isaac.ui.ui_utils import get_style, LABEL_WIDTH
from omni.isaac.ui.element_wrappers import CollapsableFrame, FloatField

class AbstractHeuristic:
    def __init__(self):
        self.name = ""

    def apply_heuristic(self, original_csv_path, metric_csv_path, output_csv_path):
        pass

    def get_name(self):
        return self.name
    
    def config_pane(self, pane, wrapped_ui_elements):
        pass

class ShaveUnusedHeuristic(AbstractHeuristic):
    def __init__(self):
        self.name = "Shave Unused Sensors"

    def apply_heuristic(self, original_csv_path, metric_csv_path, output_csv_path):

        # Import the metric data
        metric_data = self.import_metric_csv(metric_csv_path)

        # We want to set a threshold of contact points that prevents sensors from being removed
        # if threshold is 0, then we remove all sensors that have never been contacted
        threshold = 0

        remaining_indices = []
        for sensor in metric_data:
            if int(sensor[1]) > threshold:
                remaining_indices.append(sensor[0])

        # Import the original sensor data to shave off the unused sensors
        original_data = self.import_original_csv(original_csv_path, remaining_indices)

        # Export the output data
        self.export_output_csv(output_csv_path, original_data)

    def import_metric_csv(self, path):
        """
        Function that imports the metric data from a CSV file
        CSV file should have the following format:
        Sensor Name, contact count
        """
        data = np.genfromtxt(path, delimiter=',', skip_header=1, dtype=str)
        return data

    def import_original_csv(self, path, idices):
        """
        Function that imports the sensor data from a CSV file
        CSV file should have the following format:
        Sensor Name, X Offset, Y Offset, Z Offset, Norm X, Norm Y, Norm Z, Radius, Parent Path
        Only reads the lines where sensor name == index
        """

        new_index = 0

        with open(path, 'r') as f:
            lines = f.readlines()
            sensors = []
            for line in lines:
                data = line.split(',')
                if data[0] in idices:
                    data[0] = str(new_index)
                    sensors.append(data)
                    new_index += 1
        return sensors
    
    def export_output_csv(self, path, sensors):
        with open(path, 'w') as f:
            f.write('Sensor Name, X Offset, Y Offset, Z Offset, Norm X, Norm Y, Norm Z, Radius, Parent Path\n')
            for sensor in sensors:
                f.write(','.join(sensor))

    def config_pane(self, pane, wrapped_ui_elements):
        print("Configuring Shave Heuristic Pane")
        with pane:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self.threshold_field = FloatField(
                    "Threshold", 
                    default_value=0, 
                    lower_limit=0, 
                    upper_limit=1000,
                    on_value_changed_fn=self.on_threshold_changed
                    )
                #wrapped_ui_elements.append(self.threshold_field)

    def on_threshold_changed(self, value):
        self.threshold = value

class InterpolateHeatmapHeuristic(AbstractHeuristic):
    def __init__(self):
        self.name = "Interpolate Heatmap"
        self.bw_coeff = 2.0
        self.act_dist = 0.5 # in meters
        self.wrapped_ui_elements = []

    class VertexData:
        def __init__(self, index, weight=0, obj_name="", pos=None, contact_count=0):
            self.index = index
            self.weight = weight
            self.obj_name = obj_name
            self.pos = pos
            self.contact_count = contact_count

    def apply_heuristic(self, vertices_path, metric_csv_path, output_csv_path):
        # This heuristic will assign a weight to all vertices and normalize to create a heatmap
        
        # Import the metric data (contact count)
        # In the format of [Index, Contact Count, X, Y, Z, Object Name]
        metric_data = self.import_metric_csv(metric_csv_path)

        # Import the vertices data
        # In the format of [Index, X, Y, Z, Object Name]
        vertices_data = self.import_vertices_csv(vertices_path)

        # Loop through the vertices and add a weight to each vertex based on the contact count
        # Each assigned weight will first pass through a filter that contrsaints the weight to a range of 0 to 1
        new_vertices = self.BW_filter(vertices_data, metric_data)

        # Export the output data
        self.export_output_csv(output_csv_path, new_vertices)
        
    def BW_filter(self, vertices, metrics):
        # Example filter of weights based on contact count
        max_weight = {}

        for v in vertices:
            for m in metrics:
                if v.obj_name == m.obj_name: # Only apply weights where the names match
                    distance = Gf.Vec3f(v.pos - m.pos).GetLength()

                    v.weight = np.maximum(v.weight, np.sqrt(float(m.contact_count)/ (1+np.abs((1/(self.act_dist)) * distance) ** (2*self.bw_coeff)))) # Low Pass Butterworth Filter

                    #print(f"Comparing V{v.index} and M{m.index} with distance {distance}: Weight = {v.weight}")

                    if v.obj_name not in max_weight:
                        max_weight[v.obj_name] = v.weight
                    elif v.weight > max_weight[v.obj_name]:
                        max_weight[v.obj_name] = v.weight

        #Normalize the weights
        for v in vertices:
            v.weight = (v.weight / max_weight[v.obj_name])
            #v.weight = np.clip(v.weight, 0, 1)

        return vertices

    def import_metric_csv(self, path):
        data = np.genfromtxt(path, delimiter=',', skip_header=1, dtype=str)
        metric_data = []

        for row in data:
            if float(row[1]) == 0:
                continue
            md = self.VertexData(row[0], contact_count=row[1], pos=Gf.Vec3f(float(row[2]), float(row[3]), float(row[4])), obj_name=row[5])
            metric_data.append(md)

        return metric_data
    
    def import_vertices_csv(self, path):
        data = np.genfromtxt(path, delimiter=',', skip_header=1, dtype=str)
        vertices_data = []

        for row in data:
            vd = self.VertexData(row[0], pos=Gf.Vec3f(float(row[1]), float(row[2]), float(row[3])), obj_name=row[4])
            vertices_data.append(vd)

        return vertices_data
    
    def export_output_csv(self, path, new_vertices):
        # Outputs a heatmap of the vertices
        with open(path, 'w') as f:
            f.write('Index, Weight, Object Name\n')
            for vertex in new_vertices:
                f.write(f"{vertex.index},{vertex.weight},{vertex.obj_name}\n")
    
    def config_pane(self, pane, wrapped_ui_elements):
        print("Configuring Interpolate Heatmap Heuristic Pane")
        with pane:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self.bw_coeff_field = FloatField(
                    "Butterworth Coefficient", 
                    default_value=2.0, 
                    lower_limit=0, 
                    upper_limit=1000,
                    on_value_changed_fn=self.on_bw_coeff_changed
                    )
                #wrapped_ui_elements.append(self.bw_coeff_field)

                self.activation_distance_field = FloatField(
                    "Activation Distance (m)",
                    default_value=0.5,
                    lower_limit=0,
                    upper_limit=1, 
                    on_value_changed_fn=self.on_act_dist_factor_changed)
                #wrapped_ui_elements.append(self.activation_distance_field)
    
    def on_bw_coeff_changed(self, value):
        self.bw_coeff = value

    def on_act_dist_factor_changed(self, value):
        self.act_dist = value

# Make sure to add all new heuristic classes to the heruistic tracker list
class HeuristicTracker:
    def __init__(self):
        self.heuristics = []

        ######### Add new heuristics here #########
        self.heuristics.append(InterpolateHeatmapHeuristic())
        self.heuristics.append(ShaveUnusedHeuristic())

        ############################################

    def get_heuristic(self, name):
        for heuristic in self.heuristics:
            if heuristic.get_name() == name:
                return heuristic
        return None

    def get_heuristics(self):
        return self.heuristics

    def apply_heuristic(self, name, original_csv_path, metric_csv_path, output_csv_path):
        heuristic = self.get_heuristic(name)
        if heuristic:
            heuristic.apply_heuristic(original_csv_path, metric_csv_path, output_csv_path)
        else:
            print("Heuristic not found")

    def update_config_pane(self, name, pane, wrapped_ui_elements):
        heuristic = self.get_heuristic(name)
        if heuristic:
            heuristic.config_pane(pane, wrapped_ui_elements)
        else:
            print("Heuristic not found")

