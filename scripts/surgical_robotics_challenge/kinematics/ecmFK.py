import numpy as np
from surgical_robotics_challenge.utils.utilities import *
from surgical_robotics_challenge.kinematics.DH import *


# THIS IS THE FK FOR THE ECM THIS IS THE
# SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL.
class ECMKinematicData:
    def __init__(self):
        self.num_links = 5
        # Setting these to zero as we don't really need them here.
        self.L_rcc = 0.0
        self.L_scopelen = 0.0

        # PSM DH Params
        # alpha | a | theta | d | offset | type | convention
        self.kinematics = [DH(PI_2, 0, 0, 0, PI_2, JointType.REVOLUTE, Convention.MODIFIED),
                           DH(-PI_2, 0, 0, 0, -PI_2, JointType.REVOLUTE, Convention.MODIFIED),
                           DH(PI_2, 0, 0, 0, -self.L_rcc, JointType.PRISMATIC, Convention.MODIFIED),
                           DH(0, 0, 0, self.L_scopelen, 0, JointType.REVOLUTE, Convention.MODIFIED),
                           DH(PI, 0, 0, 0, PI_2, JointType.REVOLUTE, Convention.MODIFIED)] # LAST DH TO COMPENSATION THE FRAME OFFSET

    def get_link_params(self, link_num):
        if link_num < 0 or link_num > self.num_links:
            # Error
            print("ERROR, ONLY ", self.num_links, " JOINT DEFINED")
            return []
        else:
            return self.kinematics[link_num]


kinematics_data = ECMKinematicData()


# You need to provide a list of joint positions. If the list is less that the number of joint
# i.e. the robot has 6 joints, but only provide 3 joints. The FK till the 3+1 link will be provided
def compute_FK(joint_pos, up_to_link):
    if up_to_link > kinematics_data.num_links:
        raise "ERROR! COMPUTE FK UP_TO_LINK GREATER THAN DOF"
    j = [0, 0, 0, 0, 0]
    for i in range(len(joint_pos)):
        j[i] = joint_pos[i]

    T_N_0 = np.identity(4)

    for i in range(up_to_link):
        link_dh = kinematics_data.get_link_params(i)
        link_dh.theta = j[i]
        T_N_0 = T_N_0 * link_dh.get_trans()

    return T_N_0
