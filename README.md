### How to Run

1. Clone, build and source AMBF's `restructure` branch.

https://github.com/WPI-AIM/ambf/tree/restructure

```bash
git checkout -b restructure origin/restructure
git pull
```

2. Clone this repo outside AMBF source tree e.g. your home folder.
Lets call the location of this folder as
`<surgical_robotics_challenge>`

3. Now run AMBF with the launch file and ADFs from this repo as:

```
cd <ambf_bin>
./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,13,14
```

This is an example of what the scene should look like (minus the motions of the PSM, Needle etc.):


<p align="center">
<img src=Media/sample_scene.gif width="480"/>
</p>

4. To control the PSMs, you can run the following script in a new terminal:
```
cd <surgical_robotics_challenge>/scripts/
python gui_based_control.py
```
You should see GUI's with sliders to control the Pose of each PSM.
Move around the PSMs and try to pick the needle.

5. To automatically make the needle move towards a PSMs grasp, you can run the script called `attach_needle.py` while
the simulation is running. Once the needle is within the grasping area you can control that specific jaw angle (using the GUI launched above) until it grasps the needle.

6. You can press `CTRL+R` to reset the simulation. Press `P` key to toggle between mouse pan using the LEFT click or mouse picking.
