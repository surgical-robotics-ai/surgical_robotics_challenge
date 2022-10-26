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
import PyKDL
from PyKDL import Vector, Rotation
from surgical_robotics_challenge.utils.utilities import cartesian_interpolate_step
import numpy as np
import time
import rospy
import sys
if sys.version_info[0] >= 3:
    from tkinter import *
else:
    from Tkinter import *


class NeedleOffsets:
    TnINt1 = PyKDL.Frame(Rotation.RPY(-np.pi/2., 0., np.pi),
                   Vector(-0.10727960616350174, -0.07585766911506653, -0.013998392969369888))
    TnINt2 = PyKDL.Frame(Rotation.RPY(-np.pi/2., 0., 0.),
                   Vector(0.09973019361495972, -0.05215135216712952, 0.03237169608473778))
    TnINt3 = PyKDL.Frame(Rotation.RPY(-np.pi/2., 0., 0.),
                   Vector(0.10727960616350174, -0.07585766911506653, -0.013998392969369888))


def attach_needle(needle, link, T_offset):
    reached = False
    if link is None:
        print('Not a valid link, returning')
        return
    T_nINw = needle.get_pose()
    while not reached and not rospy.is_shutdown():
        T_tINw = link.get_pose()
        T_nINw_cmd = T_tINw * T_offset

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
        needle.set_pose(T_cmd)
        time.sleep(0.001)
        # T_nINw = get_obj_trans(needle)

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


simulation_manager = SimulationManager('attach_needle')
# psm_name =
needle = simulation_manager.get_obj_handle('Needle')
link1 = simulation_manager.get_obj_handle('psm1' + '/toolyawlink')
link2 = simulation_manager.get_obj_handle('psm2' + '/toolyawlink')
link3 = simulation_manager.get_obj_handle('psm3' + '/toolyawlink')
time.sleep(0.5)

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
