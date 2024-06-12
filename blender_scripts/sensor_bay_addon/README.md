# Blender Tactile Sensor Bay Add-on
The blender tactile sensor bay addon is a tool made for procedurally generating variable density 3D printed tactile skins. This tool was designed to streamline the process of making large-scale coverage tactile sensor arrays.

# Importing the Add-on:
![Alt Text](demos/install_addon.gif)
1) Either clone this repo or download **sensor_bay_addon.zip** under the *blender_scripts* folder.
2) Navigate to the Add-on window in Blender under *Edit > Preferences > Add-ons*
3) Click the *Install* button and navigate to the local save location of **sensor_bay_addon.zip**
4) In the search bar in the preferences window, search for **Tactile Sensor Bay** and enable the add-on. If the add-on does not show up, make sure the *Enabled Add-ons Only* button is not on.

# Applying the Skin Geometry Node:


# Saving Configurations
![Alt Text](demos/saving.gif)
The *Save Sensor Positions* button exports a .csv file contatining the sensor node positions as well as their respective path in the scene. Make sure the root primitive of the robot is selected before clicking the save button. This file is used to import the sensors designed in this environment to the Isaac Sim extension. [Click here](https://github.com/cKohl10/TactileSim/tree/main/exts) to learn more about how to use the [**Isaac Sim Contact Extension**](https://github.com/cKohl10/TactileSim/tree/main/exts) 