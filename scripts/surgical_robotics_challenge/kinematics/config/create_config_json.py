import numpy as np
import os
import sys
dynamic_path = os.path.abspath(__file__ + "/../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from glob import glob
import json





def create_config_json(psm_type='classic', tool_id=None):
    assert type(tool_id) == int, "tool id must be an integer, please check the input"
    pass


if __name__ == "__main__":
    # file_folder = os.path.join(dynamic_path, 'kinematics', 'config')
    file_folder = os.path.join(dynamic_path)