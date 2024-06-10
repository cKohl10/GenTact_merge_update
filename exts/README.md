This extension has been tested on **Isaac Sim verison 4.0.0**. Using other versions may cause instability.

# To add the Contact Extension to Isaac Sim:

  1: Open the extensions window, *Window* > *Extensions*

  2: Add the path to extenstions in the *Extension Search Paths* window in the form `/TactileSim/exts`

  3: Under *Third Party* in the extenstions window, toggle the **Contact Extension**. It is recommended to also enable autoloading to skip this process upon reopening Isaac Sim.
  
  4: Open the extension from the toolbar at the top of the screen.

# To use the extension:

  1: After creating a CSV list of sensor positions, radius, and paths as described in `/TactileSim/blender_scripts/sensor_bay_addon/README.md`

  2: IMPORTANT: For a new robot, a contact sensor has to first be manually added to each link that will be covered. 
  - This allows the link to become the parent of a contact sensor. After manually adding the sensor once, it can be deleted and the robot can be saved. 

  3: Open the extension by clicking the **Contact Extension** tab in the top toolbar.

  4: Use the file explorer button to the right of the *Import CSV File* option to locate the folder containing sensor configuration CSVs made using the [Blender Sensor Bay Addon](https://github.com/cKohl10/TactileSim/tree/main/blender_scripts/sensor_bay_addon). 
  - Note: This field is a string and can be manually entered. The dropdown called *Config Options* can be used to navigate through folders until you choose a **.csv** file.

  5: Ensure the robot exists in the scene under the same path as the CSV's *Parent Path* component.

  6: Click **Update** to recurrsively create contact sensors. The *Sensor Readings* panel should populate with a configurable heatmap representing each sensor in the order of which they were applied.
  - Known Error: If the heatmap is not displaying and data, try saving the robot with the sensors applied and reopening the USD file.

**Trouble Shooting**: Please reach out to carson.kohlbrenner@gmail.com if you encounter problems with this extension. The source code is provided for this extension under *exts > contact_ext > Contact_Extension_Test_Python*