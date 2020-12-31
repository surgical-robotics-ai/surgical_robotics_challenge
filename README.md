### How to Run

1. Clone, build and source AMBF's `ambf-1.0` branch.

https://github.com/WPI-AIM/ambf/tree/ambf-1.0

```bash
git checkout origin/ambf-1.0
git checkout -b ambf-1.0
git pull origin ambf-1.0
```

2. Clone this repo outside AMBF source tree. Maybe your home folder.
Lets call the root of this repo as
`<surgical_robotics_challenge>`

3. Now run AMBF with the launch file and ADFs from this repo as:

```
cd <ambf_bin>
./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,6,7,9,10
```

You should see three PSMs, a sad needle with fake thread on a side table, and a Gynae mesh.

4. To control the PSMs, you can use the following script. In a new terminal
```
cd <surgical_robotics_challenge>/scripts/
python gui_based_control.py
```
You should see GUI's with sliders to control the Pose of each PSM.
Move around the PSMs and try to pick the needle. To see how to programmatically
control the PSMs and pick the needle using sensor/constraints, look at the `gui_based_control.py` code.

5. To automatically make a PSM grasp the needle, you can look at and run the script called `attach_needle.py` while
the simulation is running. Its a simple script that should move the needle near the pinchers and then you can move that
specific pinchers jaw angle until it grasps the needle.

6. You can hit `CTRL+R` to reset the simulation. Toggle `P` on the keyboard to select between mouse pan using the LEFT click or mouse picking.
