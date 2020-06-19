#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2019
#     (aimlab.wpi.edu)

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

#     \author    <aimlab.wpi.edu>
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================
from psmIK import *
from ambf_client import Client
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
import obj_control_gui


def test_ik():
    # We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
    # in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)
    test_q = [-0.3, 0.2, 0.1, -0.9, 0.0, 0.0, 0.0]
    T_7_0 = compute_FK(test_q)
    computed_q = compute_IK(convert_mat_to_frame(T_7_0))
    for i in range(0, 6):
        computed_q[i] = round(computed_q[i], 4)

    print('Test Q: ', test_q[0:6])
    print('Comp Q: ', computed_q)


class TestPSMIK:
    def __init__(self, run_psm_one, run_psm_two, OG1=None, OG2=None):
        self.c = Client()
        self.c.connect()

        self.base1 = None
        self.base2 = None
        self.target1 = None
        self.target2 = None
        self.run_psm_one = run_psm_one
        self.run_psm_two = run_psm_two

        self.obj_gui1 = None
        self.obj_gui2 = None

        time.sleep(1.0)

        if self.run_psm_one is True:
            print('PREPARING TO LOAD IK FOR PSM1')
            self.base1 = self.c.get_obj_handle('psm1/baselink')
            self.target1 = self.c.get_obj_handle('psm1/target')
            self.obj_gui1 = OG1
            base_pos = self.base1.get_pos()
            print "Base Pos 1"
            print base_pos
            init_xyz = [base_pos.x, base_pos.y, base_pos.z-0.15]
            self.obj_gui1.set_init_xyz(init_xyz)

        if self.run_psm_two is True:
            print('PREPARING TO LOAD IK FOR PSM2')
            self.base2 = self.c.get_obj_handle('psm2/baselink')
            self.target2 = self.c.get_obj_handle('psm2/target')
            self.obj_gui2 = OG2
            base_pos = self.base2.get_pos()
            print "Base Pos 2"
            print base_pos
            init_xyz = [base_pos.x, base_pos.y, base_pos.z-0.15]
            self.obj_gui2.set_init_xyz(init_xyz)

        if not run_psm_one and not run_psm_two:
            print('YOU HAVE TO RUN ATLEAST ONE PSMS IK FOR THIS SCRIPT TO DO ANYTHING')

        # The following are the names of the controllable joints.
        #  'baselink-yawlink', 0
        #  'yawlink-pitchbacklink', 1
        #  'pitchendlink-maininsertionlink', 2
        #  'maininsertionlink-toolrolllink', 3
        #  'toolrolllink-toolpitchlink', 4
        #  'toolpitchlink-toolgripper1link', 5a
        #  'toolpitchlink-toolgripper2link', 5b

    def run(self):
        while not rospy.is_shutdown():

            if self.run_psm_one is True:
                self.obj_gui1.App.update()
                # Move the Target Position Based on the GUI
                x = self.obj_gui1.x
                y = self.obj_gui1.y
                z = self.obj_gui1.z
                ro = self.obj_gui1.ro
                pi = self.obj_gui1.pi
                ya = self.obj_gui1.ya
                gr = self.obj_gui1.gr
                self.target1.set_pos(x, y, z)
                self.target1.set_rpy(ro, pi, ya)

                p = self.base1.get_pos()
                q = self.base1.get_rot()
                P_b_w = Vector(p.x, p.y, p.z)
                R_b_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
                T_b_w = Frame(R_b_w, P_b_w)
                p = self.target1.get_pos()
                q = self.target1.get_rot()
                P_t_w = Vector(p.x, p.y, p.z)
                R_t_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
                T_t_w = Frame(R_t_w, P_t_w)
                T_t_b = T_b_w.Inverse() * T_t_w
                computed_q = compute_IK(T_t_b)

                # print('SETTING JOINTS: ')
                # print(computed_q)

                self.base1.set_joint_pos('baselink-yawlink', computed_q[0])
                self.base1.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
                self.base1.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
                self.base1.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
                self.base1.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
                self.base1.set_joint_pos('toolpitchlink-toolgripper1link', -computed_q[5]+gr)
                self.base1.set_joint_pos('toolpitchlink-toolgripper2link', computed_q[5]+gr)

            if self.run_psm_two is True:
                self.obj_gui2.App.update()
                # Move the Target Position Based on the GUI
                x = self.obj_gui2.x
                y = self.obj_gui2.y
                z = self.obj_gui2.z
                ro = self.obj_gui2.ro
                pi = self.obj_gui2.pi
                ya = self.obj_gui2.ya
                gr = self.obj_gui2.gr
                self.target2.set_pos(x, y, z)
                self.target2.set_rpy(ro, pi, ya)

                p = self.base2.get_pos()
                q = self.base2.get_rot()
                P_b_w = Vector(p.x, p.y, p.z)
                R_b_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
                T_b_w = Frame(R_b_w, P_b_w)
                p = self.target2.get_pos()
                q = self.target2.get_rot()
                P_t_w = Vector(p.x, p.y, p.z)
                R_t_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
                T_t_w = Frame(R_t_w, P_t_w)
                T_t_b = T_b_w.Inverse() * T_t_w
                computed_q = compute_IK(T_t_b)

                # print('SETTING JOINTS: ')
                # print(computed_q)

                self.base2.set_joint_pos('baselink-yawlink', computed_q[0])
                self.base2.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
                self.base2.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
                self.base2.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
                self.base2.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
                self.base2.set_joint_pos('toolpitchlink-toolgripper1link', -computed_q[5]+gr)
                self.base2.set_joint_pos('toolpitchlink-toolgripper2link', computed_q[5]+gr)

            time.sleep(0.005)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print parsed_args

    OG1 = None
    OG2 = None

    if parsed_args.run_psm_one is True:
        # Initial Target Offset for PSM1
        psm1_xyz = [0.0, 0.0, 0.0]
        psm1_rpy = [3.14, 0, -1.57079]
        OG1 = obj_control_gui.ObjectGUI('psm1/baselink', psm1_xyz, psm1_rpy, 1.0, 3.14, 0.0001)

    if parsed_args.run_psm_two is True:
        # Initial Target Offset for PSM2
        psm2_xyz = [0.0, 0.0, 0.0]
        psm2_rpy = [3.14, 0, -1.57079]
        OG2 = obj_control_gui.ObjectGUI('psm2/baselink', psm2_xyz, psm2_rpy, 1.0, 3.14, 0.0001)

    psmIK = TestPSMIK(parsed_args.run_psm_one, parsed_args.run_psm_two, OG1, OG2)
    psmIK.run()
