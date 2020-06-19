#!/bin/sh
cd ./scripts/
gnome-terminal --tab --execute python testIK.py

cd ~/ambf/ambf_ros_modules/ambf_comm/examples/object_control_example
gnome-terminal --tab --execute python control_object.py -c True -j False -a psm1_target_ctrl -o psm1/target --ixyz 0.5,0.5141,-0.8297 --irpy 3.14,0,-1.57079
gnome-terminal --tab --execute python control_object.py -c True -j False -a psm2_target_ctrl -o psm2/target --ixyz 0.1245,0.5141,-0.8297 --irpy 3.14,0,-1.57079
