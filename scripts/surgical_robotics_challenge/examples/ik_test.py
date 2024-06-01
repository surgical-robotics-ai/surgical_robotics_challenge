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

from surgical_robotics_challenge.kinematics.psmKinematics import PSMKinematicSolver
from surgical_robotics_challenge.utils.joint_space_trajectory_generator import JointSpaceTrajectory
from surgical_robotics_challenge.utils.utilities import *
from surgical_robotics_challenge.kinematics.psmKinematics import ToolType
import numpy as np


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


num_joints = 7
joint_lims = np.zeros((num_joints, 2))
joint_lims[0] = [np.deg2rad(-91.96), np.deg2rad(91.96)]
joint_lims[1] = [np.deg2rad(-60), np.deg2rad(60)]
joint_lims[2] = [0.2, 2.4]
joint_lims[3] = [np.deg2rad(-175), np.deg2rad(175)]
joint_lims[4] = [np.deg2rad(-90), np.deg2rad(90)]
joint_lims[5] = [np.deg2rad(-85), np.deg2rad(85)]
joint_lims[6] = [0.0, 0.0]
js_traj = JointSpaceTrajectory(
    num_joints=7, num_traj_points=50, joint_limits=joint_lims)
num_points = js_traj.get_num_traj_points()
num_joints = 6
tool_id = ToolType.Default
psm_solver = PSMKinematicSolver(psm_type=tool_id, tool_id=tool_id)
for i in range(num_points):
    test_q = js_traj.get_traj_at_point(i)
    T_7_0 = psm_solver.compute_FK(test_q, 7)

    computed_q = psm_solver.compute_IK(convert_mat_to_frame(T_7_0))

    test_q = round_vec(test_q)
    T_7_0 = round_mat(T_7_0, 4, 4, 3)
    errors = [0] * num_joints
    for j in range(num_joints):
        errors[j] = test_q[j] - computed_q[j]
    print(i, ': Joint Errors from IK Solver')
    error_str = ""
    for i in range(len(errors)):
        errors[i] = round(errors[i], 2)
        if errors[i] == 0.0:
            error_str = error_str + " " + bcolors.OKGREEN + \
                str(errors[i]) + bcolors.ENDC
        else:
            error_str = error_str + " " + bcolors.FAIL + \
                str(errors[i]) + bcolors.ENDC
    # print(bcolors.WARNING + "errors" + bcolors.ENDC)
    print(error_str)
