# Surgical Robotics Challenge
For more information regarding the challenge, please visit [Surgical Robotics Challenge 2021-2022](https://surgical-robotics-ai.github.io/surgical-robotics-challenge/challenge-2021.html) or [Surgical Robotics Challenge 2023-2024](https://surgical-robotics-ai.github.io/surgical-robotics-challenge-2023/challenge-2023.html).

# [Discussions Forum](https://github.com/surgical-robotics-ai/surgical_robotics_challenge/discussions)
Please checkout the [Discussions Tab](https://github.com/surgical-robotics-ai/surgical_robotics_challenge/discussions) for asking questions, posting suggestions, connecting with the community, and for keeping up to date with the challenge.

# 1. Install AMBF and ROS Prerequisites
Clone, build and source `ambf-3.0` using these [instructions](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF)


# 2. Clone this repo to your local machine OR use a Dockerfile

#### Option 1: (Clone repo to your local machine)
 Please refer to [README](./scripts/README.md) in the [scripts](./scripts) folder for instructions on installing the Python package for system-wide access.

#### Option 2: (Use Dockerfile)
 You can alternatively use Dockerfiles to create Docker images by following the instructions here:
 https://github.com/surgical-robotics-ai/docker_surgical_robotics_challenge


# 3. Running the simulation
 For convenience, we have provided several bash (`.sh`) scripts to launch different suturing scenes/setups.
 
 **FOR ROS 1 `roscore` MUST BE RUNNIG BEFOREHAND. NOT REQUIRED FOR ROS 2**
 
 To run roscore, open a new terminal and run:
```bash
roscore
```
Then, you can run the `run_env_LND_420006.sh`, for example, in your terminal as:
  
 ```bash
  ./run_env_LND_420006.sh
 ```
  
 and you should see the following scene

  <p align="center">
  <img src=Media/3d_med_phantom_with_420006_psms.png width="480"/>
  </p>

### 3a. The launch file:
 To understand the launch file, refer to this [link](https://github.com/WPI-AIM/ambf/wiki/Selecting-Robots)


### 3b. Simulated Cameras
 The simulated camera(s) are defined in the World file ([`world_stereo.yaml`](./ADF/world/world_stereo.yaml)) which is set in the [`launch.yaml`](./launch.yaml) file.
 To enable the camera(s) to publish the scene image or depth data, follow the [instructions](https://github.com/WPI-AIM/ambf/wiki/Camera-feed-and-depth-camera) on this page:

### 3c. Camera Coordinate frames
 Camera coordinate frames and the difference between the AMBF and the `OpenCV` camera convention is described in [camera_convention.md](./docs/camera_conventions.md)

### 3c. Resetting the Simulation
 You can press `CTRL+R` to reset the rigid bodies in simulation, and `CTRL+V` to reset the camera pose.

### 3d. Launch Arguments:
 To manually control what objects are spawing in the scene, please review the `.sh` scripts in this folder. For a full list of arguments to provide to AMBF, please refer to these [instructions](https://github.com/WPI-AIM/ambf/wiki/Command-Line-Arguments)


# 4. Interacting with Simulated Robots using Python Scripts:
Please take a look at the scripts in the [`scripts`](./scripts) folder:


# 5. Controlling via Input Devices
The code in the scripts folder allows the dVRK MTMs or Geomagic Touch / Phantom Omni to control the simulated PSMs.

With the simulation already running, run the `dvrk-ros` application for the `dVRK MTMs` or the ROS application for the `Geomagic Touch/Phantom Omni`. You can find the relevant code for them here:

**a. https://github.com/jhu-dvrk/dvrk-ros** (dvrk-ros)

**b. https://github.com/WPI-AIM/ros_geomagic** (geomagic_touch/phantom_omni)

Then run one of the corresponding Python scripts:

**a. scripts/surgical_robotics_challenge/teleoperation/mtm_multi_psm_control.py** (For MTMs)

**b. scripts/surgical_robotics_challenge/geomagic_multi_psm_control.py** (For Geomagic Touch/Phantom Omni)

Refer to the `README` in the scripts folder for further information

# 6. Citation
If you find this work useful, please cite it as:

```bibtex
@article{munawar2022open,
  title={Open Simulation Environment for Learning and Practice of Robot-Assisted Surgical Suturing},
  author={Munawar, Adnan and Wu, Jie Ying and Fischer, Gregory S and Taylor, Russell H and Kazanzides, Peter},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={2},
  pages={3843--3850},
  year={2022},
  publisher={IEEE}
}
```
