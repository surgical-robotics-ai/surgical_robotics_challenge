### INFO

### Installation:

Please install this package so that it is accessible externally.
In a terminal, navigate to this folder:
```
cd surgical_robotics_challenge/scripts/
```
Depending upon your Python version

For Python 2
```
pip install -e .
```

For Python 3

```
pip3 install -e .
```

### Description:
The module names should be self-evident.

Here is the description of some of the scripts.

### 1. Wrappers for simulation components
1. `psm_arm.py`: Wraps the simulated PSMs (in AMBF simulation) using their ROS topics.
2. `ecm_arm.py`: Wraps the simulated ECM (in AMBF simulation) using its ROS topics.
3. `scene.py`: Wraps the simulated needle, entry, and exit holes (in AMBF simulation) using their ROS topics.
4. `launch_crtk_interface.py`: Spawns CRTK based ROS topics for each simulated PSM, ECM, and scene objects (needle, entry, and exits). By using this script, the PSMs and ECM can be controlled via CRTK compatible ROS topics.

### 2.Kinematics
1. `psmIK.py`: Analytical Inverse Kinematics for the PSM arm
2. `ecmIK.py`: Analytical Inverse Kinematics for the ECM arm
3. `psmFK.py`: Forward Kinematics for the PSM arm


### 3. Examples
1. `gui_based_control.py`: Uses GUI-based sliders to control the Cartesian pose of the PSMs.
2. `depth_sub.py`: Example showing a ROS subscriber for a camera depth message.
3. `image_sub.py`: Example showing a ROS subscriber for a camera image message.
4. `crtk_ros_based_control.py`: Example showcasing the control of PSM using the CRTK-ROS interface.
5. `ecm_control.py`: Example showcasing the control of ECM using the CRTK-method interface.
6. `attach_needle.py`: A GUI that allows a user to move a needle to be within the grasp of a relevant PSM.

### 4. Teleoperation
1. `mtm_device.py`: Wraps the MTM device using its ROS topics **(OLD, sawIntuitiveResearchKit < 2.0 branch)**
2. `mtm_device_crtk.py`: Wraps the MTM device using its CRTK based ROS topics **(Current, sawIntuitiveResearchKit 2.0+)**
3. `mtm_multi_psm_control.py`: Uses the `mtm_device.py` and multiple `psm_arm.py` to bind a single MTM to multiple PSMs. Only one PSM is controllable at a time and the next PSM can be selected by quickly double-tapping the clutch foot pedal. Run the script with `-h` to see allowed command-line options.
4. `geomagic_device.py`: Wraps the Geomagic device using its ROS topics
5. `geomagic_multi_psm_control.py`: Uses the `geomagic_device.py` and multiple `psm_arm.py` to bind a single Geomagic to multiple PSMs. Only one PSM is controllable at a time and the next PSM can be selected by quickly double-tapping the grey button on the device. Run the script with `-h` to see allowed command-line options.
