import numpy as np
from utilities import *
from kinematics import *


# THIS IS THE FK FOR THE PSM MOUNTED WITH THE LARGE NEEDLE DRIVER TOOL. THIS IS THE
# SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL. NOTE, JUST LIKE A FAULT IN THE
# MTM's DH PARAMETERS IN THE MANUAL, THERE IS A FAULT IN THE PSM's DH AS WELL. BASED ON
# THE FRAME ATTACHMENT IN THE DVRK MANUAL THE CORRECT DH CAN FOUND IN THIS FILE

# ALSO, NOTICE THAT AT HOME CONFIGURATION THE TIP OF THE PSM HAS THE FOLLOWING
# ROTATION OFFSET W.R.T THE BASE. THIS IS IMPORTANT FOR IK PURPOSES.
# R_7_0 = [ 0,  1,  0 ]
#       = [ 1,  0,  0 ]
#       = [ 0,  0, -1 ]
# Basically, x_7 is along y_0, y_7 is along x_0 and z_7 is along -z_0.

# You need to provide a list of joint positions. If the list is less that the number of joint
# i.e. the robot has 6 joints, but only provide 3 joints. The FK till the 3+1 link will be provided

class PSMKinematicData:
    def __init__(self):
        self.num_links = 7

        self.L_rcc = 4.389 # From dVRK documentation x 10
        self.L_tool = 4.16  # From dVRK documentation x 10
        self.L_pitch2yaw = 0.09  # Fixed length from the palm joint to the pinch joint
        self.L_yaw2ctrlpnt = 0.106  # Fixed length from the pinch joint to the pinch tip
        self.L_tool2rcm_offset = 0.229  # Delta between tool tip and the Remote Center of Motion

        # PSM DH Params
        # alpha | a | theta | d | offset | type
        self.kinematics = [[PI_2, 0, 0, 0, PI_2, 'R', 'MODIFIED'],
                           [-PI_2, 0, 0, 0, -PI_2, 'R', 'MODIFIED'],
                           [PI_2, 0, 0, 0, -self.L_rcc, 'P', 'MODIFIED'],
                           [0, 0, 0, self.L_tool, 0, 'R', 'MODIFIED'],
                           [-PI_2, 0, 0, 0, -PI_2, 'R', 'MODIFIED'],
                           [-PI_2, self.L_pitch2yaw, 0, 0, -PI_2, 'R', 'MODIFIED'],
                           [-PI_2, 0, 0, self.L_yaw2ctrlpnt, PI_2, 'R', 'MODIFIED']]

    def get_link_params(self, link_num):
        if link_num < 0 or link_num > self.num_links:
            # Error
            print("ERROR, ONLY ", self.num_links, " JOINT DEFINED")
            return []
        else:
            return self.kinematics[link_num]


kinematics_data = PSMKinematicData()


def compute_FK(joint_pos):
    j = [0, 0, 0, 0, 0, 0, 0]
    for i in range(len(joint_pos)):
        j[i] = joint_pos[i]

    T_N_0 = np.identity(4)

    for i in range(len(joint_pos)):
        link_dh = kinematics_data.get_link_params(i)
        link_N = DH(alpha=link_dh[0],
                    a=link_dh[1],
                    theta=joint_pos[i],
                    d=link_dh[3],
                    offset=link_dh[4],
                    joint_type=link_dh[5],
                    convention=link_dh[6])
        T_N_0 = T_N_0 * link_N.get_trans()

    return T_N_0


# T_7_0 = compute_FK([-0.5, 0, 0.2, 0, 0, 0])
#
# print(T_7_0)
# print("\n AFTER ROUNDING \n")
# print(round_mat(T_7_0, 4, 4, 3))
