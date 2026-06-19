# AMBF Surgical Robotics Challenge

This repository provides an open simulation environment for robotic suturing.

Please check out the [Discussions tab](https://github.com/surgical-robotics-ai/surgical_robotics_challenge/discussions) to ask questions, post suggestions, connect with the community, and stay up to date with the challenge.

## Installation

1. Install ROS2 (any version should do). Instructions for Jazzy can be found [here](https://docs.ros.org/en/jazzy/Installation.html).

2. Clone, build, and source `ambf-3.0` using these [instructions](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF). AMBF 3.0 also requires ROS, so skip the ROS-related instructions if you followed step 1 and vice-versa.

3. Set up this repository using one of the following options:
  1. Local install (recommended): Refer to the [README](./scripts/README.md) in the [scripts](./scripts) folder for instructions on installing the Python package.
  2. Docker: Create Docker images by following the instructions [here](https://github.com/surgical-robotics-ai/docker_surgical_robotics_challenge).

## Running the simulation

1. Open a terminal and set up for ROS 2, either in every terminal window that interacts with SRC or once in your `.bashrc` file, as described [here](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF#step-3).

2. Navigate to the `surgical_robotics_challenge` folder, which is `~/surgical_robotics_challenge` if you cloned it in your home directory.

3. Run one of the environments below in your terminal:

a. 3D Med Complex Suturing Phantom

```bash
./run_env_3D_MED_COMPLEX_LND_420006.sh
```

b. 3D Med Simple Suturing Phantom

```bash
./run_env_3D_MED_STRAIGHT_LND_420006.sh
```

c. Simple Suturing Phantom

```bash
./run_env_SIMPLE_LND_420006.sh
```

d. Asymmetric Pegboard with Wall

```bash
./run_env_pegboard_asymmetric.sh
```

e. Symmetric Pegboard with Wall

```bash
./run_env_pegboard_symmetric_with_wall.sh
```

f. Legacy Simple Phantom

```bash
./run_env_SIMPLE_LND_420006.sh
```

Example media showing a few different environments.

  <p align="center">
  <img src=Media/figure_eight.gif width="600"/>
  </p>

  <p align="center">
  <img src=Media/3d_med_complex_rim_light.png width="600"/>
  </p>

  <p align="center">
  <img src=Media/3d_med_straight_rim_light.png width="600"/>
  </p>

  <p align="center">
  <img src=Media/pegboard_symmetric_rim_light.png width="600"/>
  </p>


## Technical Details

### Launch file
To understand the launch file, refer to this [link](https://github.com/WPI-AIM/ambf/wiki/Selecting-Robots).

### Simulated Cameras
The simulated camera(s) are defined in the world file ([`world_stereo.yaml`](./ADF/world/world_stereo.yaml)), which is selected in [`launch.yaml`](./launch.yaml).
To enable the camera(s) to publish scene images or depth data, follow the [instructions](https://github.com/WPI-AIM/ambf/wiki/Camera-feed-and-depth-camera) on this page.

### Camera Coordinate frames
Camera coordinate frames, and the difference between the AMBF and `OpenCV` camera conventions, are described in [camera_conventions.md](./docs/camera_conventions.md).

### Resetting the Simulation
You can press `CTRL+R` to reset the rigid bodies in the simulation and `CTRL+V` to reset the camera pose.

### Launch Arguments
To manually control which objects are spawned in the scene, review the `.sh` scripts in this folder. For a full list of arguments that can be passed to AMBF, refer to these [instructions](https://github.com/WPI-AIM/ambf/wiki/Command-Line-Arguments).

## Citation
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
