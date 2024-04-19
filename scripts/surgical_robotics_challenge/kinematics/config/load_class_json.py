import numpy as np
from surgical_robotics_challenge.utils.utilities import *
from surgical_robotics_challenge.kinematics.DH import *
import os
import sys
dynamic_path = os.path.abspath(__file__ + "/../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from glob import glob
import json
import jsonpickle
from surgical_robotics_challenge.kinematics.psmKinematics import PSMKinematicSolver


def load_kinematic_json(root_dir=dynamic_path, psm_type='classic', tool_id=None):
    assert type(tool_id) == int, "tool id must be an integer, please check the input"
    file_name = glob(os.path.join(root_dir, f'class_psm_{psm_type}*{str(tool_id)}.json'))
    with open(file_name[0], 'r') as f:
        data = f.read()
        obj = jsonpickle.decode(data)
    return obj


if __name__ == "__main__":
    psm_ks = load_kinematic_json(tool_id=420006)

    T = psm_ks.compute_FK([0, 0, 0, 0, 0, 0], 7)
    test_k = psm_ks.kinematics
    joint_v = psm_ks.compute_IK(convert_mat_to_frame(T))