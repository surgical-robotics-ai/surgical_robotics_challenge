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

from surgical_robotics_challenge.simulation_manager import SimulationManager
from PyKDL import Vector, Rotation, Frame
from surgical_robotics_challenge.utils.utilities import cartesian_interpolate_step
import numpy as np
import time
import rospy


class NeedleInitialization:
    def __init__(self, simulation_manager):
        self.TnINt2 = Frame(Rotation.RPY(-np.pi / 2., 0., 0.),
                       Vector(0.009973019361495972, -0.005215135216712952, 0.003237169608473778))
        self.TnINt2_far = Frame(Rotation.RPY(-np.pi / 2., 0., 0.),
                       Vector(0.009973019361495972, -0.02, 0.003237169608473778))
        self.needle = simulation_manager.get_obj_handle('Needle')
        time.sleep(1.0)
        self._release = False
        self._reached = False

    def get_tip_to_needle_offset(self):
        return self.TnINt2

    def move_to(self, psm_tip):
        print('Moving Needle to PSM 2 Tip')
        self._release = False
        if psm_tip is None:
            print('Not a valid link, returning')
            return
        T_nINw = self.needle.get_pose()
        T_tINw = psm_tip.get_pose()
        # First reach the farther point
        self._reached = False
        done = False
        while not done:
            T_nINw_cmd = T_tINw * self.TnINt2_far
            T_delta, done = cartesian_interpolate_step(T_nINw, T_nINw_cmd, 0.01, 0.005)
            r_delta = T_delta.M.GetRPY()
            # print(error_max)
            T_cmd = Frame()
            T_cmd.p = T_nINw.p + T_delta.p
            T_cmd.M = T_nINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_nINw = T_cmd
            self.needle.set_pose(T_cmd)
            time.sleep(0.01)

        time.sleep(0.5)
        done = False
        T_nINw = self.needle.get_pose()
        T_tINw = psm_tip.get_pose()
        while not done:
            T_nINw_cmd = T_tINw * self.TnINt2
            T_delta, done = cartesian_interpolate_step(T_nINw, T_nINw_cmd, 0.01, 0.005)
            r_delta = T_delta.M.GetRPY()
            T_cmd = Frame()
            T_cmd.p = T_nINw.p + T_delta.p
            T_cmd.M = T_nINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_nINw = T_cmd
            self.needle.set_pose(T_cmd)
            time.sleep(0.01)

        self._reached = True

    def release(self):
        print('Releasing Needle')
        self._release = True
        self.needle.set_force(Vector(0, 0, 0))
        self.needle.set_torque(Vector(0, 0, 0))

    def has_reached(self):
        return self._reached

