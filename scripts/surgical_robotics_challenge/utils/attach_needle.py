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
from PyKDL import Vector, Rotation
import time
import rospy
import sys
if sys.version_info[0] >= 3:
    from tkinter import *
else:
    from Tkinter import *


def attach_needle(needle, link):
    error = 1000
    if link is None:
        print('Not a valid link, returning')
        return
    while error > 0.1 and not rospy.is_shutdown():
        P_tINw = Vector(link.get_pos().x,
                        link.get_pos().y,
                        link.get_pos().z)

        # R_tINw = Rotation.Quaternion(tool_yaw_link.get_rot().x,
        #                              tool_yaw_link.get_rot().y,
        #                              tool_yaw_link.get_rot().x,
        #                              tool_yaw_link.get_rot().w)

        R_tINw = Rotation.RPY(link.get_rpy()[0],
                              link.get_rpy()[1],
                              link.get_rpy()[2])

        # we need to move the needle based on the Pose of the toolyawlink.
        # The yawlink's n direction faces the grasp
        # position. Therefore, lets add a small offset to the P of yawlink.
        y_offset = Vector(-0.08, -0.1, 0)
        P_nINw = P_tINw + R_tINw * y_offset

        # If you want to rotate the needle to a certain relative orientation
        # add another Rotation and multiply on the R.H.S of R_tINw in the
        # Equation below.
        R_nINw = R_tINw * Rotation.RPY(-1.57079, 0, 3.14)

        needle.set_pos(P_nINw[0],
                       P_nINw[1],
                       P_nINw[2])
        needle.set_rot(R_nINw.GetQuaternion())
        time.sleep(0.001)
        P_tINw = Vector(link.get_pos().x,
                        link.get_pos().y,
                        link.get_pos().z)
        P_nINw = Vector(needle.get_pos().x,
                        needle.get_pos().y,
                        needle.get_pos().z)
        error = (P_tINw - P_nINw).Norm()
        print(error)

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
    attach_needle(needle, link1)


def psm2_btn_cb():
    attach_needle(needle, link2)


def psm3_btn_cb():
    attach_needle(needle, link3)


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
