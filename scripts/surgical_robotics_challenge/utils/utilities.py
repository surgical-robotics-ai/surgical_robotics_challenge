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

from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
from geometry_msgs.msg import Pose, PoseStamped
import json
import re

PI = np.pi
PI_2 = np.pi/2


# The up vector is useful to define angles > PI. Since otherwise
# this method will only report angles <= PI.
def get_angle(vec_a, vec_b, up_vector=None):
    vec_a.Normalize()
    vec_b.Normalize()
    cross_ab = vec_a * vec_b
    vdot = dot(vec_a, vec_b)
    # print('VDOT', vdot, vec_a, vec_b)
    # Check if the vectors are in the same direction
    if 1.0 - vdot < 0.000001:
        angle = 0.0
        # Or in the opposite direction
    elif 1.0 + vdot < 0.000001:
        angle = np.pi
    else:
        angle = math.acos(vdot)

    if up_vector is not None:
        same_dir = np.sign(dot(cross_ab, up_vector))
        if same_dir < 0.0:
            angle = -angle

    return angle


def round_mat(mat, rows, cols, precision=4):
    for i in range(0, rows):
        for j in range(0, cols):
            mat[i, j] = round(mat[i, j], precision)
    return mat


def round_vec(vec, precision=4):
    for i in range(3):
        vec[i] = round(vec[i], precision)
    return vec


def round_transform(mat, precision=4):
    return round_mat(mat, 4, 4, precision)


def convert_frame_to_mat(frame):
    np_mat = np.mat([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)
    for i in range(3):
        for j in range(3):
            np_mat[i, j] = frame.M[(i, j)]

    for i in range(3):
        np_mat[i, 3] = frame.p[i]

    return np_mat


def convert_mat_to_frame(mat):
    frame = Frame(Rotation.RPY(0, 0, 0), Vector(0, 0, 0))
    for i in range(3):
        for j in range(3):
            frame[(i, j)] = mat[i, j]

    for i in range(3):
        frame.p[i] = mat[i, 3]

    return frame


def rot_mat_to_quat(cp):
    R = Rotation(cp[0, 0], cp[0, 1], cp[0, 2],
                 cp[1, 0], cp[1, 1], cp[1, 2],
                 cp[2, 0], cp[2, 1], cp[2, 2])

    return R.GetQuaternion()


def np_mat_to_pose(cp):
    pose = Pose()
    pose.position.x = cp[0, 3]
    pose.position.y = cp[1, 3]
    pose.position.z = cp[2, 3]

    Quat = rot_mat_to_quat(cp)

    pose.orientation.x = Quat[0]
    pose.orientation.y = Quat[1]
    pose.orientation.z = Quat[2]
    pose.orientation.w = Quat[3]
    return pose


def np_mat_to_pose_stamped(cp):
    pose_stamped = PoseStamped()
    pose_stamped.pose = np_mat_to_pose(cp)
    return pose_stamped


def pose_to_frame(cp):
    frame = Frame()
    frame.p = Vector(cp.position.x,
                     cp.position.y,
                     cp.position.z)
    frame.M = Rotation.Quaternion(cp.orientation.x,
                                  cp.orientation.y,
                                  cp.orientation.z,
                                  cp.orientation.w)
    return frame


def pose_stamped_to_frame(cp):
    return pose_to_frame(cp.pose)


def frame_to_pose(T):
    pose = Pose()
    pose.position.x = T.p[0]
    pose.position.y = T.p[1]
    pose.position.z = T.p[2]
    q = T.M.GetQuaternion()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def frame_to_pose_stamped(T):
    pose_stamped = PoseStamped()
    pose_stamped.pose = frame_to_pose(T)
    return pose_stamped


def get_boolean_from_opt(opt):
    if opt in ['True', 'true', '1', True]:
        return True
    elif opt in ['False', 'false', '0', False]:
        return False
    else:
        print("Error: Option is invalid: ", opt)
        raise ValueError


def cartesian_interpolate_step(T_curr, T_goal, max_delta=0.01, deadband=0.01):
    error = np.zeros(6)
    pe = T_goal.p - T_curr.p
    re = (T_curr.M.Inverse() * T_goal.M).GetRPY()
    for i in range(6):
        if i < 3:
            error[i] = pe[i]
        else:
            error[i] = re[i-3]

    done = False
    error_max = max(np.abs(error))
    if error_max <= deadband:
        error_scaled = error * 0.
        done = True
    else:
        error_scaled = error / error_max

    error_scaled = error_scaled * max_delta

    T_step = Frame(Rotation.RPY(error_scaled[3], error_scaled[4], error_scaled[5]),
                                Vector(error_scaled[0], error_scaled[1], error_scaled[2]))
    return T_step, done


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


def toStr(f):
    return "{:.3f}".format(f)


def WARN_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.WARNING + val + bcolors.ENDC
    return valStr


def WARN2_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.OKCYAN + val + bcolors.ENDC
    return valStr


def OK_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.OKGREEN + val + bcolors.ENDC
    return valStr


def INFO_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.OKBLUE + val + bcolors.ENDC
    return valStr


def FAIL_STR(val):
    if type(val) != str:
        val = toStr(val)
    valStr = bcolors.FAIL + val + bcolors.ENDC
    return valStr


def load_json_dvrk(file_path:str)->dict:
    '''
    Load json files from dVRK repository
    :param file_path: json file path
    :return: a dictionary with loaded json file content
    '''
    with open(file_path) as f:
        data = f.read()
        data = re.sub("//.*?\n", "", data)
        data = re.sub("/\\*.*?\\*/", "", data)
        obj = data[data.find('{'): data.rfind('}') + 1]
        jsonObj = json.loads(obj)
    return jsonObj