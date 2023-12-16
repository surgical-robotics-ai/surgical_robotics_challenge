# Record and Replay 

The sample scripts are using MTM teleoperation and AMBF ROS topics. 

## Record

For sample recording script, rosbag record is utilized. Please check the launch file in the `bagRecord` folder

### How to Run

```bash
cd ~/surgical_robotics_challenge/scripts/surgical_robotics_challenge/examples/record_replay/bagRecord
roslaunch simple_rawdata_recorder.launch
```

You may need to modify the save path of your rosbag in the launch file line [#3](https://github.com/JackHaoyingZhou/surgical_robotics_challenge/blob/devel_2023/scripts/surgical_robotics_challenge/examples/record_replay/bagRecord/simple_rawdata_recorder.launch#L3). It requires the full path name and the default path is `record_replay/test_data/test_<date info>.bag`

## Replay

For sample replaying, all scripts are in the `bagReplay` folder

| Script Name                 | Action                                                                                                                                                                                                                                                                                                                                                                       |
|-----------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| record_replay_ambf.py       | Read the rosbag and replay the movements of PSM 1&2, assuming the data is stored at `record_replay/test_data`                                                                                                                                                                                                                                                                |
| record_replay_ambf_label.py | Read the rosbag and replay the movements of the needle, assuming the data is stored at `record_replay/test_data`. In addition, enable labelling feature, when replaying, pressing keyboard button `d` can let you record the index when pushing. Also, the linear and angular velocities are also calculated. The labelled data will be saved to `record_replay/test_output` |
| grasp_needle_active.py      | Move the PSM 1 or 2 to grasp the needle                                                                                                                                                                                                                                                                                                                                      |

