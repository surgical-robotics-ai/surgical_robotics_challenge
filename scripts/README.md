### INFO

There are many scripts in the folder and the naming should be self evident.

Here is description of the some of the scripts.

### 1. Wrappers for Simulated PSMs
Relevant Files:
1. `psmIK.py`: Analytical Inverse Kinematics for the PSM arm
2. `psmFK.py`: Forward Kinematics for the PSM arm
3. `psm_arm.py`: Wraps the simulated PSMs (in AMBF simulation) using their ROS topics.


### 2. Control PSMs arms via GUI
Relevant Files:
1. `gui_based_control.py`: Uses GUI based sliders to control the Cartesian pose of the PSMs.
2. `attach_needle.py`: A GUI that allows a user to move a needle to be within the grasp of a relevant PSM.

### 3. MTMs to Control Simulated PSMs
Relevant Files:

1. `mtm_device.py`: Wraps the MTM device using its ROS topics **(OLD, sawIntuitiveResearchKit < 2.0 branch)**
1. `mtm_device_crtk.py`: Wraps the MTM device using its CRTK based ROS topics **(Current, sawIntuitiveResearchKit 2.0)**
2. `mtm_multi_psm_control.py`: Uses the `mtm_device.py` and multiple `psm_arm.py` to bind a single MTM to multiple PSMs. Only one PSM is controllable at a time and the next PSM can be selected by quickly double tapping the clutch footpedal. Run the script with `-h` to see allowed command line options.

### 4. Geomagic/Phantom Omnis to Control Simulated PSMs
Relevant Files:

1. `geomagic_device.py`: Wraps the Geomagic device using its ROS topics
2. `geomagic_multi_psm_control.py`: Uses the `geomagic_device.py` and multiple `psm_arm.py` to bind a single Geomagic to multiple PSMs. Only one PSM is controllable at a time and the next PSM can be selected by quickly double tapping the grey button on the device. Run the script with `-h` to see allowed command line options.

### 5. CRTK Interface for the PSMs and geomagic_multi_psm_control
Relevant File(s):
1. `crtk_interface.py`: Spawns CRTK based ROS topics for each simulated PSM and ECM. By using this script, one does not need any other script in this folder to control the simulated robots, only the relevant ROS topics.
