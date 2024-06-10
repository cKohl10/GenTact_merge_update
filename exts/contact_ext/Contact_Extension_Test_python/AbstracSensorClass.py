# Abstract Sensor Class
# Author: Carson Kohlbrenner
# Date: 6/3/2024

"""
This class is an abstract class that defines the basic structure of a sensor.
"""

class AbstractSensorOperator:
    def __init__(self):
        self.sensors = {}
        self.config_path = ""
        self._status_report_field = None

    def import_sensors_fn(self):
        """
        Function that executes when the user clicks the 'Update' button
        Imports the sensor data from the CSV file and creates the sensors
        """
        pass

    def remove_sensors_fn(self):
        """
        Function that removes all sensors from the robot
        """
        pass

    def sensor_update(self, dt):
        """
        Function that updates the sensor data
        """
        pass

    def create_sensor_readings_frame(self):
        """
        Function that creates the sensor readings frame
        """
        pass

    def update_sensor_readings_frame(self):
        """
        Function that updates the sensor readings
        """
        pass