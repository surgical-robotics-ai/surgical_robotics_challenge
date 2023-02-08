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
from enum import Enum


class JointType(Enum):
    REVOLUTE = 0
    PRISMATIC = 1


class Convention(Enum):
    STANDARD = 0
    MODIFIED = 1


class DH:
    def __init__(self, alpha, a, theta, d, offset, joint_type, convention):
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d
        self.offset = offset
        self.joint_type = joint_type
        self.convention = convention

    def mat_from_dh(self, alpha, a, theta, d, offset, joint_type, convention):
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        th = 0.0
        if joint_type == JointType.REVOLUTE:
            th = theta + offset
        elif joint_type == JointType.PRISMATIC:
            d = d + offset + theta
        else:
            assert joint_type == JointType.REVOLUTE and joint_type == JointType.PRISMATIC
            return

        ct = np.cos(th)
        st = np.sin(th)

        if convention == Convention.STANDARD:
            mat = np.mat([
                [ct, -st * ca,  st * sa,  a * ct],
                [st,  ct * ca, -ct * sa,  a * st],
                [0,        sa,       ca,       d],
                [0,         0,        0,       1]
            ])
        elif convention == Convention.MODIFIED:
            mat = np.mat([
                [ct, -st, 0, a],
                [st * ca, ct * ca, -sa, -d * sa],
                [st * sa, ct * sa, ca, d * ca],
                [0, 0, 0, 1]
            ])
        else:
            raise 'ERROR, DH CONVENTION NOT UNDERSTOOD'

        return mat

    def get_trans(self):
        return self.mat_from_dh(self.alpha, self.a, self.theta, self.d, self.offset, self.joint_type, self.convention)


def enforce_limits(j_raw, lower_lims, upper_lims):
    num_joints = len(j_raw)
    j_limited = [0.0]*num_joints

    for idx in range(num_joints):
        min_lim = lower_lims[idx]
        max_lim = upper_lims[idx]
        j_limited[idx] = max(min_lim, min(j_raw[idx], max_lim))

    return j_limited
