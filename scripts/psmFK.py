import numpy as np
from utilities import *


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

class KinematicData:
    def __init__(self):
        self.num_links = 7

        self.L_pitch2yaw = 0.09  # Fixed length from the palm joint to the pinch joint
        self.L_yaw2ctrlpnt = 0.106  # Fixed length from the pinch joint to the pinch tip
        self.L_tool2rcm_offset = 0.229  # Delta between tool tip and the Remote Center of Motion

        # PSM DH Params
        # alpha | a | theta | d | offset | type
        self.kinematics = [[PI_2, 0, 0, 0, PI_2, 'R'],
                           [-PI_2, 0, 0, 0, -PI_2, 'R'],
                           [PI_2, 0, 0, 0, 4.389, 'P'],
                           [0, 0, 0, 4.16, 0, 'R'],
                           [-PI_2, 0, 0, 0, -PI_2, 'R'],
                           [-PI_2, 0.09, 0, 0, -PI_2, 'R'],
                           [-PI_2, 0, 0, 0.106, PI_2, 'R']]

    def get_link_params(self, link_num):
        if link_num < 0 or link_num > self.num_links:
            # Error
            print("ERROR, ONLY ", self.num_links, " JOINT DEFINED")
            return []
        else:
            return self.kinematics[link_num]


kinematics_data = KinematicData()


def compute_FK(joint_pos):
    j = [0, 0, 0, 0, 0, 0, 0]
    for i in range(len(joint_pos)):
        j[i] = joint_pos[i]

    # The last frame is fixed

    # PSM DH Params
    # alpha | a | theta | d | offset | type
    # all_dh_params = [[ PI_2,    0,  j[0],     0,  PI_2, 'R'],
    #                  [-PI_2,    0,  j[1],     0, -PI_2, 'R'],
    #                  [ PI_2,    0,  j[2],     0, 4.389, 'P'],
    #                  [    0,    0,  j[3],  4.16,     0, 'R'],
    #                  [-PI_2,    0,  j[4],     0, -PI_2, 'R'],
    #                  [-PI_2, 0.09,  j[5],     0, -PI_2, 'R'],
    #                  [-PI_2,    0,     0, 0.106,  PI_2, 'R']]

    # print("RETURNING FK FOR LINK ", len(joint_pos))

    T_N_0 = np.identity(4)

    for i in range(len(joint_pos)):
        link_dh = kinematics_data.get_link_params(i)
        link_N = DH(alpha=link_dh[0],
                    a=link_dh[1],
                    theta=joint_pos[i],
                    d=link_dh[3],
                    offset=link_dh[4],
                    joint_type=link_dh[5])
        T_N_0 = T_N_0 * link_N.get_trans()

    return T_N_0


class DH:
    def __init__(self, alpha, a, theta, d, offset, joint_type):
        self.alpha = alpha
        self.a = a
        self.theta = 0
        self.d = d
        self.offset = offset
        self.joint_type = joint_type

        if self.joint_type == 'R':
            self.theta = theta + offset
        elif self.joint_type == 'P':
            self.d = d + offset + theta
        else:
            assert type == 'P' and type == 'R'

    def mat_from_dh(self, alpha, a, theta, d):
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        ct = np.cos(theta)
        st = np.sin(theta)
        mat = np.mat([
            [ct     , -st     ,  0 ,  a],
            [st * ca,  ct * ca, -sa, -d * sa],
            [st * sa,  ct * sa,  ca,  d * ca],
            [0      ,  0      ,  0 ,  1]
        ])
        return mat

    def get_trans(self):
        return self.mat_from_dh(self.alpha, self.a, self.theta, self.d)


# T_7_0 = compute_FK([-0.5, 0, 0.2, 0, 0, 0])
#
# print T_7_0
# print "\n AFTER ROUNDING \n"
# print(round_mat(T_7_0, 4, 4, 3))

