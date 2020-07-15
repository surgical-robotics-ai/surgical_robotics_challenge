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
./ambf_simulator --launch_file <surgical_robotics_challenge>/ADF/launch.yaml -l 0,1,2,3,4,5,6
```

You should see two PSMs, a sad needle, a table, and a Gynae mesh.

4. To control the PSMs, you can use the following script. In a new terminal
```
cd <surgical_robotics_challenge>/scripts/
python testIK.py
```
You should see GUI's with sliders to control the Pose of each PSM using IK.
Move around the PSMs and try to pick the needle. To see how to programmatically
control the PSMs and pick the needle using sensor/constraints, look at the `testIK.py` code.
