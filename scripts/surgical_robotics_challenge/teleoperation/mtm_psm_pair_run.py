import subprocess
import os
import sys

dynamic_path = os.path.abspath(__file__+"/../")
# dynamic_path = os.path.abspath(__file__+"/../../")  # outside folder
# print(dynamic_path)
sys.path.append(dynamic_path)

if __name__ == '__main__':
    subprocess.run("python3 mtm_multi_psm_control.py -c mtml_teleop --mtm MTML --one 1 --two 0 & python3 mtm_multi_psm_control.py -c mtmr_teleop --mtm MTMR --one 0 --two 1", shell=True)