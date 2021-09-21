# 1. Install AMBF and ROS Prerequisites
Make sure that the correct version of ROS is installed and sourced on your system. For streaming the image and depth data out of AMBF, please also install the following ROS packages
- cv_bridge
- image_transport

```bash
apt-get install ros-<version>-cv-bridge ros-<version>-image-transport
```
Then, clone, build and source AMBF's `ambf-2.0` branch.


https://github.com/WPI-AIM/ambf/tree/ambf-2.0

First time cloning:
```bash
git clone https://github.com/WPI-AIM/ambf.git
git checkout -b ambf-2.0 origin/ambf-2.0
```

In case there are updates to AMBF, you can simply update your local copy by:
```bash
git pull
```

Don't forget to build the repo using the instructions on AMBF's Readme

# 2. Clone this repo to your local machine OR use a Dockerfile

#### Option 1: (Clone repo to your local machine)
  Clone this repo outside the AMBF source tree e.g. your home folder.

  ```bash
  git clone https://github.com/collaborative-robotics/surgical_robotics_challenge
  ```

  Let's call the location of this folder as
  `<surgical_robotics_challenge>`

  To make the python scripts within this repo available system wide
  ```bash
  cd <surgical_robotics_challenge>/scripts/
  python install -e .
  ```
  If you are using Python3, change `python` to `python3` in the above command

#### Option 2: (Use Dockerfile)

  You can alternatively use Dockerfiles to create Docker images by following the instructions here:

  https://github.com/collaborative-robotics/docker_surgical_robotics_challenge


# 3. Running the simulation

  The simulation is spawned in AMBF with the launch file and AMBF Description Format (ADF) files from this repo:
  The `ambf_simulator` binary resides in `ambf/bin/lin-x86_64`. You should be in that directory before running the commands below. Alternatively, you can create a soft-link to this binary.
  ```bash
  ./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,14,15
  ```
  This is an example of what the scene should look like (minus the motions of the PSM, Needle etc.):

  <p align="center">
  <img src=Media/figure_eight.gif width="480"/>
  </p>

  To launch a different scene with just the needle (without any thread), you can run:

  ```bash
  ./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,13,14
  ```

  And this is what the scene should now look like:

  <p align="center">
  <img src=Media/sample_scene.gif width="480"/>
  </p>


### 3a. The launch file:
  To understand the launch file, visit the following link:

  https://github.com/WPI-AIM/ambf/wiki/Selecting-Robots

### 3b. Simulated Cameras
  The simulated camera(s) is defined in the World file (`world_stereo.yaml`) which is set in the `launch.yaml` file.
  To enable the camera(s) to publish the scene image or depth data, follow the instructions on this page:

  https://github.com/WPI-AIM/ambf/wiki/Camera-feed-and-depth-camera

### 3c. Resetting the Simulation
  You can press `CTRL+R` to reset the rigid bodies in simulation, and `CTRL+V` to reset the camera pose if you changed it with the mouse.


# 4. Interacting with Simulated Robots using Python Scripts:
Please take a look at the scripts in the `scripts` folder:


# 5. Controlling via Input Devices
The code in the scripts folder allows the dVRK MTMs or Geomagic Touch / Phantom Omni to control the simulated PSMs.

With the simulation already running, run the `dvrk-ros` application for the `dVRK MTMs` or the ROS application for the `Geomagic Touch/Phantom Omni`. You can find the relevant code for them here:

**a. https://github.com/jhu-dvrk/dvrk-ros** (dvrk-ros)

**b. https://github.com/WPI-AIM/ros_geomagic** (geomagic_touch/phantom_omni)

Then run one of the corresponding python scripts:

**a. scripts/surgical_robotics_challenge/teleoperation/mtm_multi_psm_control.py** (For MTMs)

**b. scripts/surgical_robotics_challenge/geomagic_multi_psm_control.py** (For Geomagic Touch/Phantom Omni)

Refer to the `README` in the scripts folder for further information
