## Impotant command line
Run simulator with peg transfer puzzle:

```
ambf_simulator --launch_file launch.yaml -l 0,1,3,4,16,17,18 -a ~/Hisashi/test_puzzle.yaml 
```



Play .bag file and select certain topics:

```
rosbag play ~/Downloads/2022-08-11-22-10-12.bag --topics /MTMR/measured_cp /MTMR/measured_cv /MTMR/measured_js /MTMR/gripper/measured_js /footpedals/clutch /console/operator_present/communication_loss
```

Play bug file with both MTMR and MTML
```
rosbag play ~/Downloads/2022-08-11-22-10-12.bag --topics /MTMR/measured_cp /MTMR/measured_cv /MTMR/measured_js /MTMR/gripper/measured_js /footpedals/clutch /console/operator_present/communication_loss /MTML/measured_cp /MTML/measured_cv /MTML/measured_js /MTML/gripper/measured_js
```

Running mtm_psm control for right and left

```
python3 mtm_predict3_virtual_clutch.py -c mtmr --mtm MTMR --one 0 --two 1

python3 mtm_predict3_virtual_clutch.py -c mtml --mtm MTML --one 1 --two 0

```


Playing the bag file with Puzzle condition

```
rosbag play ~/Downloads/2022-08-11-22-10-12.bag PuzzleRed1/State:=PuzzleRed1/Command PuzzleRed2/State:=PuzzleRed2/Command PuzzleRed3/State:=PuzzleRed3/Command PuzzleRed4/State:=PuzzleRed4/Command PuzzleRed5/State:=PuzzleRed5/Command PuzzleYellow/State:=PuzzleYellow/Command

```

rosrun dvrk_robot dvrk_console_json -j console-MTML-MTMR.json -p 0.001

ambf_simulator --launch_file launch.yaml -l 0,1,3,4,16,17,18,19,20,21 --override_max_comm_freq 120 -p 120 -t 1


rosbag play ~/exp/exexp12_0 bag PuzzleRed1/State:=PuzzleRed1/Command PuzzleRed2/State:=PuzzleRed2/Command PuzzleRed3/State:=PuzzleRed3/Command PuzzleRed4/State:=PuzzleRed4/Command PuzzleRed5/State:=PuzzleRed5/Command PuzzleYellow/State:=PuzzleYellow/Command


rosbag play ~/exp/exexp11_0.bag One_shadow/State:=One_shadow/Command Two_shadow/State:=Two_shadow/Command Three_shadow/State:=Three_shadow/Command Four_shadow/State:=Four_shadow/Command Five_shadow/State:=Five_shadow/Command Six_shadow/State:=Six_shadow/Command PuzzleRed1/State:=PuzzleRed1/Command PuzzleRed2/State:=PuzzleRed2/Command PuzzleRed3/State:=PuzzleRed3/Command PuzzleRed4/State:=PuzzleRed4/Command PuzzleRed5/State:=PuzzleRed5/Command PuzzleYellow/State:=PuzzleYellow/Command


rosbag play ~/exp/exexp7_0.bag PuzzleRed1/State:=PuzzleRed1/Command PuzzleRed2/State:=PuzzleRed2/Command PuzzleRed3/State:=PuzzleRed3/Command PuzzleRed4/State:=PuzzleRed4/Command PuzzleRed5/State:=PuzzleRed5/Command PuzzleYellow/State:=PuzzleYellow/Command