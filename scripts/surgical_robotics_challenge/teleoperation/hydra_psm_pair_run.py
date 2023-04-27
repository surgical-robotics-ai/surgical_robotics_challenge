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
#     \author    <hzhou6@wpi.edu>
#     \author    Haoying(Jack) Zhou
#     \version   1.0
# */
# //==============================================================================
import os
import sys

import numpy as np

dynamic_path = os.path.abspath(__file__+"/../../../")
# print(dynamic_path)
sys.path.append(dynamic_path)

from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
import time
import rospy
import PyKDL
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from input_devices.hydra_device import HydraDevice
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from surgical_robotics_challenge.utils.utilities import get_boolean_from_opt
from surgical_robotics_challenge.utils import coordinate_frames
from threading import Thread


class ControllerInterface:
    def __init__(self, leader_l, leader_r, psm_arm_l, psm_arm_r, ecm):
        self.counter = 0
        self.leader_1 = leader_l
        self.leader_2 = leader_r
        self.psm_1 = psm_arm_l
        self.psm_2 = psm_arm_r
        self.gui = JointGUI('ECM JP', 4, ["ecm j0", "ecm j1", "ecm j2", "ecm j3"])

        self.cmd1_xyz = self.psm_1.T_t_b_home.p
        self.cmd1_rpy = None
        self.cmd2_xyz = self.psm_2.T_t_b_home.p
        self.cmd2_rpy = None
        self.T1_IK = None
        self.T2_IK = None
        self._ecm = ecm

        self._T1_c_b = None
        self._T2_c_b = None
        self._update_T_c_b = True

    def update_T_c_b(self):
        if self._update_T_c_b or self._ecm.has_pose_changed():
            self._T1_c_b = self.psm_1.get_T_w_b() * self._ecm.get_T_c_w()
            self._T2_c_b = self.psm_2.get_T_w_b() * self._ecm.get_T_c_w()
            self._update_T_c_b = False

    def update_camera_pose(self):
        self.gui.App.update()
        self._ecm.servo_jp(self.gui.jnt_cmds)

    def teleop_pair_1(self):
        twist = self.leader_1.measured_cv()
        self.cmd1_xyz = self.psm_1.T_t_b_home.p
        if not self.leader_1.clutch_button_pressed:
            delta_t = self._T1_c_b.M * twist.vel * 1.2
            self.cmd1_xyz = self.cmd1_xyz + delta_t
            self.psm_1.T_t_b_home.p = self.cmd1_xyz
        self.cmd1_rpy = self._T1_c_b.M * self.leader_1.measured_cp().M * Rotation.RPY(3.14, 0, 3.14 / 2.0)
        self.T1_IK = Frame(self.cmd1_rpy, self.cmd1_xyz)
        self.psm_1.servo_cp(self.T1_IK)
        self.psm_1.set_jaw_angle(self.leader_1.get_jaw_angle())
        self.psm_1.run_grasp_logic(self.leader_1.get_jaw_angle())

    def teleop_pair_2(self):
        twist = self.leader_2.measured_cv()
        self.cmd2_xyz = self.psm_2.T_t_b_home.p
        if not self.leader_2.clutch_button_pressed:
            delta_t = self._T2_c_b.M * twist.vel * 1.2
            self.cmd2_xyz = self.cmd2_xyz + delta_t
            self.psm_2.T_t_b_home.p = self.cmd2_xyz
        self.cmd2_rpy = self._T2_c_b.M * self.leader_2.measured_cp().M * Rotation.RPY(3.14, 0, 3.14 / 2.0)
        self.T2_IK = Frame(self.cmd2_rpy, self.cmd2_xyz)
        self.psm_2.servo_cp(self.T2_IK)
        self.psm_2.set_jaw_angle(self.leader_2.get_jaw_angle())
        self.psm_2.run_grasp_logic(self.leader_2.get_jaw_angle())

    def update_arm_pose(self):
        self.update_T_c_b()
        t1 = Thread(target=self.teleop_pair_1)
        t2 = Thread(target=self.teleop_pair_2)
        t1.start()
        t2.start()
        t1.join()
        t2.join()


    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.psm_1.target_IK is not None:
            T_t_w = self.psm_1.get_T_b_w() * self.T1_IK
            self.psm_1.target_IK.set_pose(T_t_w)

        if self.psm_2.target_IK is not None:
            T_t_w = self.psm_2.get_T_b_w() * self.T2_IK
            self.psm_2.target_IK.set_pose(T_t_w)
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
        self.update_camera_pose()
        self.update_arm_pose()
        self.update_visual_markers()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='hydra_sim_teleop')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=False)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

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

    simulation_manager = SimulationManager(parsed_args.client_name)

    cam = ECM(simulation_manager, 'CameraFrame')
    time.sleep(0.5)
    cam.servo_jp([0., 0., 0., 0.])

    controllers = []
    psm_arms = []

    if parsed_args.run_psm_one is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm1'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm1 = PSM(simulation_manager, arm_name, add_joint_errors=False)
        if psm1.is_present():
            T_psmtip_c = coordinate_frames.PSM1.T_tip_cam
            T_psmtip_b = psm1.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm1.set_home_pose(T_psmtip_b)
            psm_arms.append(psm1)

    if parsed_args.run_psm_two is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm2'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm2 = PSM(simulation_manager, arm_name, add_joint_errors=False)
        if psm2.is_present():
            T_psmtip_c = coordinate_frames.PSM2.T_tip_cam
            T_psmtip_b = psm2.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm2.set_home_pose(T_psmtip_b)
            psm_arms.append(psm2)

    if parsed_args.run_psm_three is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm3'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(simulation_manager, arm_name, add_joint_errors=False)
        if psm.is_present():
            T_psmtip_c = coordinate_frames.PSM3.T_tip_cam
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)

    if len(psm_arms) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        if parsed_args.run_psm_one is True:
            leader_l = HydraDevice(hydra_idx=0)
        if parsed_args.run_psm_two is True:
            leader_r = HydraDevice(hydra_idx=1)
        theta_base = -0.9
        theta_tip = -theta_base
        leader_l.set_base_frame(Frame(Rotation.RPY(theta_base, 0, 0), Vector(0, 0, 0)))
        leader_l.set_tip_frame(Frame(Rotation.RPY(theta_base + theta_tip, 0, 0), Vector(0, 0, 0)))
        leader_r.set_base_frame(Frame(Rotation.RPY(theta_base, 0, 0), Vector(0, 0, 0)))
        leader_r.set_tip_frame(Frame(Rotation.RPY(theta_base + theta_tip, 0, 0), Vector(0, 0, 0)))
        controller = ControllerInterface(leader_l, leader_r, psm1, psm2, cam)
        controllers.append(controller)

        rate = rospy.Rate(200)

        try:
            while not rospy.is_shutdown():
                for cont in controllers:
                        cont.run()
                rate.sleep()
        except:
            print('Exception! Goodbye')

