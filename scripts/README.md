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
The module names should be self-evident but a brief description is provided below:

### Controlling Simulated Robots:
After launching the `ambf_simulator` as described in the main [README](../README.md), there are two ways to control the simulated PSMs and ECM, and read the pose (state) of the needle and entry exit points

1. Using the scripts with CRTK Method API:
    With this option, the following scripts `psm_arm.py`, `ecm_arm.py` and `scene.py` can be imported inside an application. Each script provides a CRTK compatible method API for controlling the robot or reading relevant data (pose of entry / exits markers).
2. Using the CRTK-ROS interface:
    With this option, there is no need to import any scripts. Simply run the `launch_crtk_interface.py` script. This script will consequently use the `psm_arm.py`, `ecm_arm.py` and `scene.py` scripts and create ROS topics to publish and receive commands.


  The [examples](./surgical_robotics_challenge/examples) folder contains demonstrations of using both these ways to control the simulation.


### 1. Wrappers for simulation components
| # | Script Name                | Description                                                                                                                                                                                      |
|---|----------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 | `psm_arm.py`               | Wraps the simulated PSMs (in AMBF simulation) using their ROS topics.                                                                                                                            |
| 2 | `ecm_arm.py`               | Wraps the simulated ECM (in AMBF simulation) using its ROS topics.                                                                                                                               |
| 3 | `scene.py`                 | Wraps the simulated needle, entry, and exit holes (in AMBF simulation) using their ROS topics.                                                                                                   |
| 4 | `launch_crtk_interface.py` | Spawns CRTK based ROS topics for each simulated PSM, ECM, and scene objects (needle, entry, and exits). By using this script, the PSMs and ECM can be controlled via CRTK compatible ROS topics. |
| 5 | `camera.py`                | Provides access to a kinematic frame that is used a parent for the Simulated ECM. (Not used at the moment)   

### 2. Kinematics
| # | Script Name | Description                                   |
|---|-------------|-----------------------------------------------|
| 1 | `psmIK.py`  | Analytical Inverse Kinematics for the PSM arm |
| 2 | `ecmIK.py`  | Analytical Inverse Kinematics for the ECM arm |
| 3 | `psmFK.py`  | Forward Kinematics for the PSM arm            |
| 4 | `DH.py`     | DH implementation                             |


### 3. Examples
| # | Script Name                 | Description                                                                            |
|---|-----------------------------|----------------------------------------------------------------------------------------|
| 1 | `gui_based_control.py`      | Uses GUI-based sliders to control the Cartesian pose of the PSMs.                      |
| 2 | `depth_sub.py`              | Example showing a ROS subscriber for a camera depth message.                           |
| 3 | `image_sub.py`              | Example showing a ROS subscriber for a camera image message.                           |
| 4 | `crtk_ros_based_control.py` | Example showcasing the control of PSM using the CRTK-ROS interface.                    |
| 5 | `ecm_control.py`            | Example showcasing the control of ECM using the CRTK-method interface.                 |
| 6 | `ik_test.py`                | Tests the PSM IK implementation using a random trajectory.                             |
| 7 | `camera_frame_control.py`   | Controls the kinematic frame which is used as a parent for the ECM. (Not used for now) |

### 4. Teleoperation
| # | Script Name                     | Description                                                                                                                                                                                                                                                                                        |
|---|---------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 | `mtm_multi_psm_control.py`      | Uses the `mtm_device.py` and multiple `psm_arm.py` to bind a single MTM to multiple PSMs. Only one PSM is controllable at a time and the next PSM can be selected by quickly double-tapping the clutch foot pedal. Run the script with `-h` to see allowed command-line options.                   |
| 2 | `geomagic_multi_psm_control.py` | Uses the `geomagic_device.py` and multiple `psm_arm.py` to bind a single Geomagic to multiple PSMs. Only one PSM is controllable at a time and the next PSM can be selected by quickly double-tapping the grey button on the device. Run the script with `-h` to see allowed command-line options. |
| 3 | `razer_multi_psm_control.py`    | Uses the `razer_device.py` and multiple `psm_arm.py` to bind a single Razer Hydra to multiple PSMs. Not tested fully.        

#### 4a. Input Devices (teleoperation/input_devices)

| # | Script Name          | Description                                                                                        |
|---|----------------------|----------------------------------------------------------------------------------------------------|
| 1 | `mtm_device.py`      | Wraps the MTM device using its ROS topics **(OLD, sawIntuitiveResearchKit < 2.0 branch)**          |
| 2 | `mtm_device_crtk.py` | Wraps the MTM device using its CRTK based ROS topics **(Current, sawIntuitiveResearchKit >= 2.0)** |
| 3 | `geomagic_device.py` | Wraps the Geomagic device using its ROS topics                                                     |

### 5. Utils

Consists of various helper scripts that are used in this package.

| # | Script Name          | Description                                                                                        |
|---|----------------------|----------------------------------------------------------------------------------------------------|
| 1 | `approx_sync_data.py` | Example of collecting ros messages using approximate time synchronizer|

