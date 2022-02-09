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
