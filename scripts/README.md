## Scripts Package

This folder contains the Python package used to control and evaluate the simulation.

## Installation

Install the package in editable mode so it is importable from other locations.

Optional: create and activate a virtual environment.

```bash
cd ~
python3 -m venv venv_src --system-site-packages
source ~/venv_src/bin/activate
```

Note: Activate the virtual environment in each terminal that needs these scripts. You can also add the `source` command to your `.bashrc` if you want it loaded automatically.

Then install the package:

```bash
cd ~/surgical_robotics_challenge/scripts
python3 -m pip install -e .
```

## Control Options

After launching one of the environments as described in the main [README](../README.md), you can control the simulated PSMs/ECM and read scene state in two ways:

1. CRTK method API (import-based)
Import and use `psm_arm.py`, `ecm_arm.py`, and `scene.py` directly in your application.

2. CRTK-ROS interface
Run `launch_crtk_interface.py` to expose CRTK-compatible ROS topics backed by the same wrappers.

The [examples](./surgical_robotics_challenge/examples) folder demonstrates both approaches.

## Control via Meta Quest

After launching the simulation:

1. Enable the CRTK interface:

```bash
cd ~/surgical_robotics_challenge/scripts/surgical_robotics_challenge
python3 launch_crtk_interface.py --scene False
```

The `scene` argument is relevant for suturing environments with entry/exit holes. Set it to `False` for other environments, including the pegboard challenge.

2. Open the teleoperation directory:

```bash
cd ~/surgical_robotics_challenge/scripts/surgical_robotics_challenge/teleoperation
```

3. Run the UDP bridge:

```bash
python3 udp_crtk_bridge.py --quest-ip <ip-address> --offset-rpy 180 0 180 --swap --home
```

Replace `<ip-address>` with the master device IP (for example, Quest 3).

This bridge converts CRTK-compatible UDP JSON commands to CRTK-compatible ROS 2 topics used by AMBF.

## 1. Wrappers for Simulation Components

| # | Script Name                | Description |
|---|----------------------------|-------------|
| 1 | `psm_arm.py`               | Wraps simulated PSMs through ROS topics. |
| 2 | `ecm_arm.py`               | Wraps the simulated ECM through ROS topics. |
| 3 | `scene.py`                 | Wraps the simulated needle, entry, and exit holes through ROS topics. |
| 4 | `launch_crtk_interface.py` | Publishes CRTK-compatible ROS topics for PSMs, ECM, and scene objects. |
| 5 | `camera.py`                | Provides access to the kinematic frame used as parent for the simulated ECM (currently not used). |

## 2. Kinematics

| # | Script Name | Description |
|---|-------------|-------------|
| 1 | `psmKinematics.py` | PSM forward/inverse kinematics implementation. |
| 2 | `ecmFK.py`         | ECM forward kinematics implementation. |
| 3 | `DH.py`            | DH convention helper implementation. |

## 3. Examples

| # | Script Name                  | Description |
|---|------------------------------|-------------|
| 1 | `gui_based_control.py`       | GUI sliders for Cartesian control of PSMs. |
| 2 | `depth_sub.py`               | Example ROS subscriber for camera depth messages. |
| 3 | `image_sub.py`               | Example ROS subscriber for camera image messages. |
| 4 | `crtk_ros_based_control.py`  | PSM control via the CRTK-ROS interface. |
| 5 | `ecm_control.py`             | ECM control via the CRTK method API. |
| 6 | `ik_test.py`                 | Random-trajectory test for PSM kinematics. |
| 7 | `interface_via_method_api.py`| Minimal method-API control example. |
| 8 | `interface_via_crtk_ros_api.py` | Minimal CRTK-ROS API control example. |

## 4. Teleoperation

| # | Script Name                     | Description |
|---|---------------------------------|-------------|
| 1 | `mtm_multi_psm_control.py`      | Binds one MTM to multiple PSMs; switch active arm by double-tapping clutch pedal. |
| 2 | `geomagic_multi_psm_control.py` | Binds one Geomagic to multiple PSMs; switch active arm by double-tapping the device button. |
| 3 | `hydra_multi_psm_control.py`    | Binds one Razer Hydra to multiple PSMs. |

### 4a. Input Devices (`teleoperation/input_devices`)

| # | Script Name          | Description |
|---|----------------------|-------------|
| 1 | `mtm_device_crtk.py` | Wraps MTM using CRTK-based ROS topics (for `sawIntuitiveResearchKit >= 2.0`). |
| 2 | `geomagic_device.py` | Wraps Geomagic using ROS topics. |
| 3 | `hydra_device.py`    | Wraps Razer Hydra using ROS topics. |

## 5. Utils

Helper scripts used across this package.

| # | Script Name                     | Description |
|---|---------------------------------|-------------|
| 1 | `utils/approx_sync_data.py`     | Example for collecting ROS messages with an approximate time synchronizer. |

