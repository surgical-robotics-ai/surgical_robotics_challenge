import subprocess
import os
import sys

dynamic_path = os.path.abspath(__file__+"/../")
# dynamic_path = os.path.abspath(__file__+"/../../")  # outside folder
# print(dynamic_path)
sys.path.append(dynamic_path)

if __name__ == '__main__':
    subprocess.run("python3 hydra_multi_psm_control.py -c hydra_l_sim_teleop --one True --two False & python3 hydra_multi_psm_control.py -c hydra_r_sim_teleop --one False --two True", shell=True)