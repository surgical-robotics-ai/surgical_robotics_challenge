#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020-2021 Johns Hopkins University (JHU), Worcester Polytechnic Institute (WPI) All Rights Reserved.


#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.


#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================

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
                           DH(-PI_2, 0, 0, 0, -PI_2,
                              JointType.REVOLUTE, Convention.MODIFIED),
                           DH(PI_2, 0, 0, 0, -self.L_rcc,
                              JointType.PRISMATIC, Convention.MODIFIED),
                           DH(0, 0, 0, self.L_scopelen, 0,
                              JointType.REVOLUTE, Convention.MODIFIED),
                           DH(PI, 0, 0, 0, PI_2, JointType.REVOLUTE, Convention.MODIFIED)]  # LAST DH TO COMPENSATION THE FRAME OFFSET
        self.lower_limits = [-1.5, -1.5, -0.1, -1.57]
        self.upper_limits = [1.5, 1.5, 0.1, 1.57]

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
