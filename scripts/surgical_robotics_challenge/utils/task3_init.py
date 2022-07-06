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

from ambf_client import Client
from PyKDL import Vector, Rotation, Frame
from surgical_robotics_challenge.utils.utilities import cartesian_interpolate_step
import numpy as np
import time
import rospy


class NeedleInitialization:
    def __init__(self):
        self.TnINt2 = Frame(Rotation.RPY(-np.pi / 2., 0., 0.),
                       Vector(0.09973019361495972, -0.05215135216712952, 0.03237169608473778))
        self.TnINt2_far = Frame(Rotation.RPY(-np.pi / 2., 0., 0.),
                       Vector(0.09973019361495972, -0.2, 0.03237169608473778))
        c = Client('task3_initialization')
        c.connect()
        self.needle = c.get_obj_handle('Needle')
        self.psm_tip = c.get_obj_handle('psm2' + '/toolyawlink')
        time.sleep(1.0)
        self._release = False
        self._reached = False

    def get_tip_to_needle_offset(self):
        return self.TnINt2

    def get_obj_trans(self, obj):
        p = obj.get_pos()
        r = obj.get_rpy()
        return Frame(Rotation.RPY(r[0], r[1], r[2]),
                     Vector(p.x, p.y, p.z))

    def lock_at_tip(self):
        print('Moving Needle to PSM 2 Tip')
        self._release = False
        if self.psm_tip is None:
            print('Not a valid link, returning')
            return
        T_nINw = self.get_obj_trans(self.needle)
        # First reach the farther point
        reached_far = False
        self._reached = False
        while not reached_far:
            T_tINw = self.get_obj_trans(self.psm_tip)
            T_nINw_cmd = T_tINw * self.TnINt2_far
            T_delta, error_max = cartesian_interpolate_step(T_nINw, T_nINw_cmd, 0.01)
            r_delta = T_delta.M.GetRPY()
            # print(error_max)
            if error_max < 0.01:
                reached_far = True
                break

            T_cmd = Frame()
            T_cmd.p = T_nINw.p + T_delta.p
            T_cmd.M = T_nINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_nINw = T_cmd
            self.needle.set_pos(T_cmd.p[0], T_cmd.p[1], T_cmd.p[2])
            self.needle.set_rpy(T_cmd.M.GetRPY()[0], T_cmd.M.GetRPY()[1], T_cmd.M.GetRPY()[2])
            time.sleep(0.01)

        time.sleep(2.0)
        while not self._release and not rospy.is_shutdown():
            T_tINw = self.get_obj_trans(self.psm_tip)
            T_nINw_cmd = T_tINw * self.TnINt2
            T_delta, error_max = cartesian_interpolate_step(T_nINw, T_nINw_cmd, 0.01)
            r_delta = T_delta.M.GetRPY()
            # print(error_max)
            if error_max < 0.01:
                self._reached = True
                break

            T_cmd = Frame()
            T_cmd.p = T_nINw.p + T_delta.p
            T_cmd.M = T_nINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_nINw = T_cmd
            self.needle.set_pos(T_cmd.p[0], T_cmd.p[1], T_cmd.p[2])
            self.needle.set_rpy(T_cmd.M.GetRPY()[0], T_cmd.M.GetRPY()[1], T_cmd.M.GetRPY()[2])
            time.sleep(0.01)

    def release(self):
        print('Releasing Needle')
        self._release = True
        self.needle.set_force(0, 0, 0)
        self.needle.set_torque(0, 0, 0)

    def is_reached(self):
        return self._reached

