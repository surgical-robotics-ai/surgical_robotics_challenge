### How to Run

1. Clone, build and source AMBF's `ambf-2.0` branch.

https://github.com/WPI-AIM/ambf/tree/ambf-2.0

First time cloning:
```bash
git clone https://github.com/WPI-AIM/ambf.git
git checkout -b ambf-2.0 origin/ambf-2.0
```
Updating to the latest commit
```bash
git pull
```

2. Clone this repo outside AMBF source tree e.g. your home folder.

```bash
git clone https://github.com/adnanmunawar/surgical_robotics_challenge
```

Lets call the location of this folder as
`<surgical_robotics_challenge>`

3. Now run AMBF with the launch file and ADFs from this repo as:



```bash
./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,14,15
```
This is an example of what the scene should look like (minus the motions of the PSM, Needle etc.):


<p align="center">
<img src=Media/figure_eight.gif width="480"/>
</p>

To launch a different scene with just the needle (without any thread), you can run:

```
cd <ambf_bin>
./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,13,14
```

And this is what the scene should now look like:

<p align="center">
<img src=Media/sample_scene.gif width="480"/>
</p>




4. To control the PSMs, you can run the following script in a new terminal:
```
cd <surgical_robotics_challenge>/scripts/
python gui_based_control.py
```
You should see GUI's with sliders to control the Pose of each PSM and you can the PSMs around and try to pick the needle.

5. To automatically make the needle move towards a PSMs grasp, you can run the script called `attach_needle.py` while
the simulation is running. Once the needle is within the grasping area you can control that specific jaw angle (using the GUI launched above) until it grasps the needle.

6. You can press `CTRL+R` to reset the simulation. Press `P` key to toggle between mouse pan using the LEFT click or mouse picking.


### Pairing Input Devices to Control Simulated PSMs
The code in the scripts folder allows the dVRK MTMs or Geomagic Touch / Phantom Omni to control the simulated PSMs.
First run the AMBF simulation as described in step 3.

Next run the `dvrk-ros` application for the `dVRK MTMs` or the ROS application for the `Geomagic Touch/Phantom Omni`. Here is where you can find the relevant code for them:

**a. https://github.com/jhu-dvrk/dvrk-ros** (dvrk-ros)

**b. https://github.com/WPI-AIM/ros_geomagic** (geomagic_touch/phantom_omni)

Then run one of the corresponding python scripts:

**a. scripts/mtm_multi_psm_control.py** (For MTMs)

**b. scripts/geomagic_multi_psm_control.py** (For Geomagic Touch/Phantom Omni)

Refer to the README in the scripts folder for further information
