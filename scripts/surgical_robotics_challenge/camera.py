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
from surgical_robotics_challenge.kinematics.psmKinematics import *
from PyKDL import Frame, Rotation, Vector, Twist
import time


class Camera:
    def __init__(self, simulation_manager, name):
        self.simulation_manager = simulation_manager
        self.name = name
        self.camera_handle = self.simulation_manager.get_obj_handle(name)
        time.sleep(0.5)

        # Transform of Base in World
        self._T_c_w = None
        # Transform of World in Base
        self._T_w_c = None
        self._pose_changed = True
        self._num_joints = 6
        self._ik_solution = np.zeros([self._num_joints])
        self._last_jp = np.zeros([self._num_joints])

    def is_present(self):
        if self.camera_handle is None:
            return False
        else:
            return True

    def get_T_c_w(self):
        self._update_camera_pose()
        return self._T_c_w

    def get_T_w_c(self):
        self._update_camera_pose()
        return self._T_w_c

    def has_pose_changed(self):
        return self._pose_changed

    def set_pose_changed(self):
        self._pose_changed = True

    def _update_camera_pose(self):
        self._T_c_w = self.camera_handle.get_pose()
        self._T_w_c = self._T_c_w.Inverse()
        self._pose_changed = False

    def move_cp(self, T_c_w):
        if type(T_c_w) in [np.matrix, np.array]:
            T_c_w = convert_mat_to_frame(T_c_w)

        self.camera_handle.set_pose(T_c_w)
        self._pose_changed = True

    def move_cv(self, twist, dt):
        if type(twist) in [np.array, np.ndarray]:
            v = Vector(twist[0], twist[1], twist[2]) * dt
            w = Vector(twist[3], twist[4], twist[5]) * dt
        elif type(twist) is Twist:
            v = twist.vel * dt
            w = twist.rot * dt
        else:
            raise TypeError

        T_c_w = self.get_T_c_w()
        T_cmd = Frame(Rotation.RPY(w[0], w[1], w[2]), v)
        self.move_cp(T_c_w * T_cmd)
        pass

    def measured_cp(self):
        return self.get_T_c_w()
