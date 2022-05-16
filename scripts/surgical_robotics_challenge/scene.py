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

from PyKDL import Frame, Rotation, Vector, Twist
import numpy as np
from surgical_robotics_challenge.utils.utilities import cartesian_interpolate_step
import time


def ambf_pose_to_frame(obj):
    p = Vector(obj.get_pos().x, obj.get_pos().y, obj.get_pos().z)
    R = Rotation.Quaternion(obj.get_rot().x, obj.get_rot().y,
                            obj.get_rot().z, obj.get_rot().w)
    T = Frame(R, p)
    return T


class Scene:
    def __init__(self, client):
        self.client = client
        self._needle = self.client.get_obj_handle("Needle")
        self._entry1 = self.client.get_obj_handle("Entry1")
        self._entry2 = self.client.get_obj_handle("Entry2")
        self._entry3 = self.client.get_obj_handle("Entry3")
        self._entry4 = self.client.get_obj_handle("Entry4")
        self._exit1 = self.client.get_obj_handle("Exit1")
        self._exit2 = self.client.get_obj_handle("Exit2")
        self._exit3 = self.client.get_obj_handle("Exit3")
        self._exit4 = self.client.get_obj_handle("Exit4")
        time.sleep(0.1)

    def needle_measured_cp(self):
        return ambf_pose_to_frame(self._needle)

    def entry1_measured_cp(self):
        return ambf_pose_to_frame(self._entry1)

    def entry2_measured_cp(self):
        return ambf_pose_to_frame(self._entry2)

    def entry3_measured_cp(self):
        return ambf_pose_to_frame(self._entry3)

    def entry4_measured_cp(self):
        return ambf_pose_to_frame(self._entry4)

    def exit1_measured_cp(self):
        return ambf_pose_to_frame(self._exit1)

    def exit2_measured_cp(self):
        return ambf_pose_to_frame(self._exit2)

    def exit3_measured_cp(self):
        return ambf_pose_to_frame(self._exit3)

    def exit4_measured_cp(self):
        return ambf_pose_to_frame(self._exit4)

    def task_3_setup_init(self, psm2):
        print("METHOD Based: Task 3 Setup Called")
        TnINt2 = Frame(Rotation.RPY(-np.pi / 2., 0., 0.),
                            Vector(0.09973019361495972, -0.05215135216712952, 0.03237169608473778))
        TnINt2_far = Frame(Rotation.RPY(-np.pi / 2., 0., 0.),
                                Vector(0.09973019361495972, -0.2, 0.03237169608473778))
        psm2_tip = self.client.get_obj_handle('psm2' + '/toolyawlink')
        time.sleep(0.5)
        release = False
        reached = False

        # First we shall move the PSM to its initial pose using joint commands OR pose command
        psm2.servo_jp([-0.4, -0.22, 1.39, -1.64, -0.37, -0.11])
        # Open the Jaws
        psm2.set_jaw_angle(0.8)
        # Sleep to achieve the target pose and jaw angle
        time.sleep(0.5)

        print('Moving Needle to PSM 2 Tip')
        release = False
        if psm2_tip is None:
            print('Not a valid link, returning')
            return
        T_nINw = ambf_pose_to_frame(self._needle)
        # First reach the farther point
        reached_far = False
        reached = False
        while not reached_far:
            T_tINw = ambf_pose_to_frame(psm2_tip)
            T_nINw_cmd = T_tINw * TnINt2_far
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
            self._needle.set_pos(T_cmd.p[0], T_cmd.p[1], T_cmd.p[2])
            self._needle.set_rpy(T_cmd.M.GetRPY()[0], T_cmd.M.GetRPY()[1], T_cmd.M.GetRPY()[2])
            time.sleep(0.01)

        time.sleep(2.0)
        while not release:
            T_tINw = ambf_pose_to_frame(psm2_tip)
            T_nINw_cmd = T_tINw * TnINt2
            T_delta, error_max = cartesian_interpolate_step(T_nINw, T_nINw_cmd, 0.01)
            r_delta = T_delta.M.GetRPY()
            # print(error_max)
            if error_max < 0.01:
                reached = True
                break

            T_cmd = Frame()
            T_cmd.p = T_nINw.p + T_delta.p
            T_cmd.M = T_nINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_nINw = T_cmd
            self._needle.set_pos(T_cmd.p[0], T_cmd.p[1], T_cmd.p[2])
            self._needle.set_rpy(T_cmd.M.GetRPY()[0], T_cmd.M.GetRPY()[1], T_cmd.M.GetRPY()[2])
            time.sleep(0.01)
        time.sleep(1.0)
        for i in range(30):
            # Close the jaws to grasp the needle
            # Calling it repeatedly a few times so that the needle is forced
            # between the gripper tips and grasped properly
            psm2.set_jaw_angle(0.0)
            time.sleep(0.01)

        time.sleep(0.5)
        print('Releasing Needle')
        release = True
        self._needle.set_force(0, 0, 0)
        self._needle.set_torque(0, 0, 0)
