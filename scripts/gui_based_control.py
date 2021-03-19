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
import obj_control_gui


class GUIController:
    def __init__(self, arm, gui_handle):
        self.counter = 0
        self.GUI = gui_handle
        self.arm = arm

    def update_arm_pose(self):
        gui = self.GUI
        gui.App.update()
        T_t_b = Frame(Rotation.RPY(gui.ro, gui.pi, gui.ya), Vector(gui.x, gui.y, gui.z))
        self.arm.move_cp(T_t_b)
        self.arm.set_jaw_angle(gui.gr)
        self.arm.run_grasp_logic(gui.gr)

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.arm.target_IK is not None:
            gui = self.GUI
            T_ik_w = self.arm.get_T_b_w() * Frame(Rotation.RPY(gui.ro, gui.pi, gui.ya), Vector(gui.x, gui.y, gui.z))
            self.arm.target_IK.set_pos(T_ik_w.p[0], T_ik_w.p[1], T_ik_w.p[2])
            self.arm.target_IK.set_rpy(T_ik_w.M.GetRPY()[0], T_ik_w.M.GetRPY()[1], T_ik_w.M.GetRPY()[2])
        if self.arm.target_FK is not None:
            ik_solution = self.arm.get_ik_solution()
            ik_solution = np.append(ik_solution, 0)
            T_t_b = convert_mat_to_frame(compute_FK(ik_solution))
            T_t_w = self.arm.get_T_b_w() * T_t_b
            self.arm.target_FK.set_pos(T_t_w.p[0], T_t_w.p[1], T_t_w.p[2])
            self.arm.target_FK.set_rpy(T_t_w.M.GetRPY()[0], T_t_w.M.GetRPY()[1], T_t_w.M.GetRPY()[2])

    def run(self):
            self.update_arm_pose()
            self.update_visual_markers()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='ambf_client')
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

    c = Client(parsed_args.client_name)
    c.connect()

    controllers = []
    
    if parsed_args.run_psm_one is True:
        arm_name = 'psm1'
        psm = PSM(c, arm_name)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM1
            # init_xyz = [0.1, -0.85, -0.15]
            init_xyz = [0, 0, -1.0]
            init_rpy = [3.14, 0, 1.57079]
            gui = obj_control_gui.ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 3.0, 10.0, 0.000001)
            controller = GUIController(psm, gui)
            controllers.append(controller)

    if parsed_args.run_psm_two is True:
        arm_name = 'psm2'
        psm = PSM(c, arm_name)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM2
            init_xyz = [0, 0.0, -1.0]
            init_rpy = [3.14, 0, 1.57079]
            gui = obj_control_gui.ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 3.0, 12, 0.000001)
            controller = GUIController(psm, gui)
            controllers.append(controller)

    if parsed_args.run_psm_three is True:
        arm_name = 'psm3'
        psm = PSM(c, arm_name)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM2
            init_xyz = [0, 0.0, -1.0]
            init_rpy = [3.14, 0, 1.57079]
            gui = obj_control_gui.ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 3.0, 3.14, 0.000001)
            controller = GUIController(psm, gui)
            controllers.append(controller)

    if len(controllers) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        while not rospy.is_shutdown():
            for cont in controllers:
                cont.run()
            time.sleep(0.005)

