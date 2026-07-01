# Initiate Surgical Robot Challenge Scenario

```bash
cd surgical_robot_challenge
./run_env_3D_MED_COMPLEX_LND_420006.sh
```

# Initiate `venv_ambf` and Run Teleoperation

```bash
source ~/venv_ambf/bin/activate
```

# Run Teleoperation

```bash
cd ~/ros2_ws/src/surgical_robotics_challenge/scripts/surgical_robotics_challenge/teleoperation
./mtm_psm_pair_teleop_420006.sh
```

# Initiate dVRK Robot

```bash
ros2 run dvrk_robot dvrk_system -j /home/xsun97/ros2_ws/install/dvrk_config_jhu/share/jhu-daVinci/system-MTML-MTMR.json
```

```text
system-SUJ-ECM-PSM1-PSM2.json
```

Change the configuration file to run other robot arms.

# If Not Homing, Try

```bash
qladisp 0 1
```

Check connection.

```bash
qladisp 2 3
```

Check connection.

```bash
qlacommand -c reset-encoder-preload
```

Restart connection.

```bash
qlacommand -c close-relays
```

# Initiate Endoscopic Camera

```bash
cd ~/ros2_ws/install/dvrk_video/share/dvrk_video/launch
ros2 launch dvrk_video decklink_stereo_1280x1024.launch.py stereo_rig_name:=jhu_daVinci
```

# Stereo Camera Calibration

```bash
ros2 run camera_calibration cameracalibrator -c jhu_daVinci --approximate 0.1 --size 8x6 --square 0.05 --no-service-check --ros-args --remap right:=/jhu_daVinci/right/image_raw --remap left:=/jhu_daVinci/left/image_raw
```

Make sure the paper checkerboard does not deform or sink when touched. Move the checkerboard around and rotate it to get at least 67 images.

Ground truth: 1600 focal length x, y.

Rectification matrix is identity matrix.

Re-projection error should be converted to mm in world frame.
