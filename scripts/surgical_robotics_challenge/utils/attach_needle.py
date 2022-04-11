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
import PyKDL
from PyKDL import Vector, Rotation
import numpy as np
import time
import rospy
import sys
if sys.version_info[0] >= 3:
    from tkinter import *
else:
    from Tkinter import *


def get_obj_trans(obj):
    P = Vector(obj.get_pos().x, obj.get_pos().y, obj.get_pos().z)
    R = Rotation.RPY(obj.get_rpy()[0], obj.get_rpy()[1], obj.get_rpy()[2])
    return PyKDL.Frame(R, P)

class NeedleOffsets:
    TnINt1 = PyKDL.Frame(Rotation.RPY(-np.pi/2., 0., np.pi),
                   Vector(-0.10727960616350174, -0.07585766911506653, -0.013998392969369888))
    TnINt2 = PyKDL.Frame(Rotation.RPY(-np.pi/2., 0., 0.),
                   Vector(0.10374952107667923, -0.07630234956741333, -0.01911688223481178))
    TnINt3 = PyKDL.Frame(Rotation.RPY(-np.pi/2., 0., 0.),
                   Vector(0.10727960616350174, -0.07585766911506653, -0.013998392969369888))


def attach_needle(needle, link, T_offset):
    reached = False
    if link is None:
        print('Not a valid link, returning')
        return
    while not reached and not rospy.is_shutdown():

        T_tINw = get_obj_trans(link)

        T_nINw_cmd = T_tINw * T_offset

        needle.set_pos(T_nINw_cmd.p[0], T_nINw_cmd.p[1], T_nINw_cmd.p[2])
        needle.set_rot(T_nINw_cmd.M.GetQuaternion())
        time.sleep(0.001)
        P_nINw_cmd = (get_obj_trans(link) * T_offset).p
        P_nINw_cur = get_obj_trans(needle).p

        error = (P_nINw_cmd - P_nINw_cur).Norm()
        print(error)
        if error < 0.001:
            reached = True

    # Wait for the needle to get there
    time.sleep(3)

    # You should see the needle in the center of the two fingers.
    # If the gripper is not already closed, you shall have to manually
    # close it to grasp the needle. You should probably automate this in the testIK script.

    # Don't forget to release the pose command from the needle. We can
    # do so by calling:
    needle.set_force(0, 0, 0)
    needle.set_torque(0, 0, 0)


def psm1_btn_cb():
    attach_needle(needle, link1, NeedleOffsets.TnINt1)


def psm2_btn_cb():
    attach_needle(needle, link2, NeedleOffsets.TnINt2)


def psm3_btn_cb():
    attach_needle(needle, link3, NeedleOffsets.TnINt3)


c = Client('attach_needle')
c.connect()
# psm_name =
needle = c.get_obj_handle('Needle')
link1 = c.get_obj_handle('psm1' + '/toolyawlink')
link2 = c.get_obj_handle('psm2' + '/toolyawlink')
link3 = c.get_obj_handle('psm3' + '/toolyawlink')

tk = Tk()
tk.title("Attache Needle")
tk.geometry("250x250")
link1_button = Button(tk, text="PSM 1", command=psm1_btn_cb,
                      height=3, width=50, bg="red")
link2_button = Button(tk, text="PSM 2", command=psm2_btn_cb,
                      height=3, width=50, bg="green")
link3_button = Button(tk, text="PSM 3", command=psm3_btn_cb,
                      height=3, width=50, bg="blue")

link1_button.pack()
link2_button.pack()
link3_button.pack()

tk.mainloop()
