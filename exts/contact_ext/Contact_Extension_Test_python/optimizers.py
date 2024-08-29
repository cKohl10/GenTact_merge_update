# Define heuristics for the optimizer here

import numpy as np
from pxr import Gf

class AbstractHeuristic:
    def __init__(self):
        self.name = ""

    def apply_heuristic(self, original_csv_path, metric_csv_path, output_csv_path):
        pass

    def get_name(self):
        return self.name

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

class InterpolateHeatmapHeuristic(AbstractHeuristic):
    def __init__(self):
        self.name = "Interpolate Heatmap"

    def apply_heuristic(self, vertices_path, metric_csv_path, output_csv_path):
        # This heuristic will assign a weight to all vertices and normalize to create a heatmap
        pass

# Make sure to add all new heuristic classes to the heruistic tracker list
class HeuristicTracker:
    def __init__(self):
        self.heuristics = []

        ######### Add new heuristics here #########
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