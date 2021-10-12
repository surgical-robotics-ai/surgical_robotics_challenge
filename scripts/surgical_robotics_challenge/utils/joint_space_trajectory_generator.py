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


class JointSpaceTrajectory:
    def __init__(self, num_traj_points, num_joints, joint_limits=None):
        self._num_traj_points = num_traj_points
        self._num_joints = num_joints
        self._traj_points = np.random.uniform(-1.0, 1.0,
                                              size=(self._num_traj_points, self._num_joints))

        if joint_limits is not None:
            joint_biases = [0]*self._num_joints
            joint_ranges = [0]*self._num_joints
            for i in range(self._num_joints):
                joint_biases[i] = joint_limits[i][0]
                joint_ranges[i] = joint_limits[i][1] - joint_limits[i][0]

            for j in range(self._num_traj_points):
                for k in range(self._num_joints):
                    normalized_val = (1.0 + self._traj_points[j, k])/2.0
                    self._traj_points[j, k] = joint_biases[k] + \
                        normalized_val * joint_ranges[k]

    def print_trajectory(self):
        print(self._traj_points)

    def get_num_traj_points(self):
        return self._num_traj_points

    def get_num_joints(self):
        return self._num_joints

    def get_traj_at_point(self, point):
        if point < self._num_traj_points:
            return self._traj_points[point]
        else:
            raise ValueError
