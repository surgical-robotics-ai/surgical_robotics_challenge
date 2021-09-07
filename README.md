### How to Run

1. Clone, build and source AMBF's `ambf-2.0` branch.

https://github.com/WPI-AIM/ambf/tree/ambf-2.0

First time cloning:
```bash
git clone https://github.com/WPI-AIM/ambf.git
cd ambf
git checkout -b ambf-2.0 origin/ambf-2.0
```
Updating to the latest commit
```bash
git pull
```

2. Clone this repo outside AMBF source tree e.g. your home folder.

```bash
git clone https://github.com/JackHaoyingZhou/surgical_robotics_challenge.git
cd surgical_robotics_challenge
git checkout Task_1
```
Updating to the latest commit
```bash
git pull
```

Lets call the location of this folder as
`<surgical_robotics_challenge>`

3. Make sure you have build and source AMBF correctly.

4. Now run AMBF with the launch file and ADFs from this repo as:


```bash
./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 14,15
```
This is an example of what the scene should look like (for visual perception task):


<p align="center">
<img src=Media/ambf_task1.png width="480"/>
</p>

5. Go to the folder `<surgical_robotics_challenge>/scripts`, run the converter between ROS image topic and opencv:

```bash
python3 camera_test.py
```

And this is what the scene should now look like:

<p align="center">
<img src=Media/ambf_cv_task1.png width="480"/>
</p>

6. Go to the folder `<surgical_robotics_challenge>/scripts`, we can obtain the point cloud data with depth:

```bash
python3 depth_sub.py
```

7. You can move the camera using your mouse; you can also zoom in/out using your mouse wheel. You can obtain the pose of camera as:

```bash
rostopic echo /ambf/env/cameras/cameraL/State
```


### How to setup ROS image topics with Python3

1. For AMBF, please make sure you have installed the following two rospackages. If not, please install it and rebuild the AMBF.

```bash
cv-bridge # Can be installed via apt install ros-<version>-cv-bridge
image-transport # Can be installed via apt install ros-<version>-image-transport
```

2. Fix ROS basic set up issue.

Please install the following packages:

```bash
sudo apt install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
```

3. Fix cv_bridge issue fix

Firstly, install the following packages:


```bash
sudo apt install python-catkin-tools python3-dev python3-numpy
```

Then, build a workspace:

```bash
mkdir ~/catkin_build_ws && cd ~/catkin_build_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
```

Clone official vision_opencv repo:

```bash
mkdir src
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
```

Finally build and source the package:

```bash
cd ~/catkin_build_ws
catkin build cv_bridge
source ./install/setup.bash --extend
```

Now, you should be able to use cv_bridge from Python3.


<!-- ### How to setup ROS image topics with Python3 -->