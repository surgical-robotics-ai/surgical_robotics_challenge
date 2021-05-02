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
from psm_arm import PSM
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from geomagic_device import GeomagicDevice


class ControllerInterface:
    def __init__(self, leader, psm):
        self.counter = 0
        self.leader = leader
        self.psm = psm

        self.init_xyz = Vector(0.0, 0.0, -1.0)
        self.init_rpy = Rotation.RPY(3.14, 0.0, 1.57079)

        self.cmd_xyz = self.init_xyz
        self.cmd_rpy = self.init_rpy

        self.T_IK = None

    def update_arm_pose(self):
        twist = self.leader.measured_cv()
        if not self.leader.gripper_button_pressed:
            self.cmd_xyz = self.cmd_xyz + twist.vel * 0.00005

        rot_offset_correction = Rotation.RPY(0.0, 0.0, 0.0)
        self.cmd_rpy = self.leader.measured_cp().M * self.init_rpy

        self.T_IK = Frame(self.cmd_rpy, self.cmd_xyz)

        self.psm.move_cp(self.T_IK)
        self.psm.set_jaw_angle(self.leader.get_jaw_angle())
        self.psm.run_grasp_logic(self.leader.get_jaw_angle())

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.psm.target_IK is not None:
            T_t_w = self.psm.get_T_b_w() * self.T_IK
            self.psm.target_IK.set_pos(T_t_w.p[0], T_t_w.p[1], T_t_w.p[2])
            self.psm.target_IK.set_rpy(T_t_w.M.GetRPY()[0], T_t_w.M.GetRPY()[1], T_t_w.M.GetRPY()[2])
        # if self.arm.target_FK is not None:
        #     ik_solution = self.arm.get_ik_solution()
        #     ik_solution = np.append(ik_solution, 0)
        #     T_7_0 = convert_mat_to_frame(compute_FK(ik_solution))
        #     T_7_w = self.arm.get_T_b_w() * T_7_0
        #     P_7_0 = T_7_w.p
        #     RPY_7_0 = T_7_w.M.GetRPY()
        #     self.arm.target_FK.set_pos(P_7_0[0], P_7_0[1], P_7_0[2])
        #     self.arm.target_FK.set_rpy(RPY_7_0[0], RPY_7_0[1], RPY_7_0[2])

    def run(self):
            self.update_arm_pose()
            self.update_visual_markers()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=True)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print parsed_args

    if parsed_args.run_psm_one in ['True', 'true', '1']:
        parsed_args.run_psm_one = True
    elif parsed_args.run_psm_one in ['False', 'false', '0']:
        parsed_args.run_psm_one = False

    if parsed_args.run_psm_two in ['True', 'true', '1']:
        parsed_args.run_psm_two = True
    elif parsed_args.run_psm_two in ['False', 'false', '0']:
        parsed_args.run_psm_two = False
    if parsed_args.run_psm_three in ['True', 'true', '1']:
        parsed_args.run_psm_three = True
    elif parsed_args.run_psm_three in ['False', 'false', '0']:
        parsed_args.run_psm_three = False

    c = Client()
    c.connect()

    controllers = []
    
    if parsed_args.run_psm_one is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm3'
        print('LOADING CONTROLLER FOR ', arm_name)
        leader = GeomagicDevice('/Geomagic/')
        theta = -0.7
        leader.set_base_frame(Frame(Rotation.RPY(theta, 0, 0), Vector(0, 0, 0)))
        leader.set_tip_frame(Frame(Rotation.RPY(theta+0.7, 0, 0), Vector()))
        psm = PSM(c, arm_name)
        controller = ControllerInterface(leader, psm)
        controllers.append(controller)

    if len(controllers) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        while not rospy.is_shutdown():
            for cont in controllers:
                cont.run()
            time.sleep(0.005)

