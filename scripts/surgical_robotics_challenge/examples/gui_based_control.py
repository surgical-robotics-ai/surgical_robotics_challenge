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
from surgical_robotics_challenge.kinematics.psmKinematics import *
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from surgical_robotics_challenge.utils.obj_control_gui import ObjectGUI
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.utilities import get_boolean_from_opt


class PSMController:
    def __init__(self, gui_handle, arm):
        self.counter = 0
        self.GUI = gui_handle
        self.arm = arm

    def update_arm_pose(self):
        gui = self.GUI
        gui.App.update()
        T_t_b = Frame(Rotation.RPY(gui.ro, gui.pi, gui.ya), Vector(gui.x, gui.y, gui.z))
        self.arm.servo_cp(T_t_b)
        self.arm.set_jaw_angle(gui.gr)
        self.arm.run_grasp_logic(gui.gr)

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.arm.target_IK is not None:
            gui = self.GUI
            T_ik_w = self.arm.get_T_b_w() * Frame(Rotation.RPY(gui.ro, gui.pi, gui.ya), Vector(gui.x, gui.y, gui.z))
            self.arm.target_IK.set_pose(T_ik_w)
        if self.arm.target_FK is not None:
            ik_solution = self.arm.get_ik_solution()
            ik_solution = np.append(ik_solution, 0)
            T_t_b = convert_mat_to_frame(self.arm.compute_FK(ik_solution))
            T_t_w = self.arm.get_T_b_w() * T_t_b
            self.arm.target_FK.set_pose(T_t_w)

    def run(self):
            self.update_arm_pose()
            self.update_visual_markers()


class ECMController:
    def __init__(self, gui, ecm):
        self.counter = 0
        self._ecm = ecm
        self._cam_gui = gui

    def update_camera_pose(self):
        self._cam_gui.App.update()
        self._ecm.servo_jp(self._cam_gui.jnt_cmds)

    def run(self):
            self.update_camera_pose()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='ambf_client')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=False)
    parser.add_argument('--ecm', action='store', dest='run_ecm', help='Control ECM', default=True)
    parser.add_argument('--tool_id', action='store', dest='psm_tool_id', help='PSM Tool ID', default=ToolType.Default)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    parsed_args.run_psm_one = get_boolean_from_opt(parsed_args.run_psm_one)
    parsed_args.run_psm_two = get_boolean_from_opt(parsed_args.run_psm_two)
    parsed_args.run_psm_three = get_boolean_from_opt(parsed_args.run_psm_three)
    parsed_args.run_ecm = get_boolean_from_opt(parsed_args.run_ecm)
    tool_id = int(parsed_args.psm_tool_id)
    simulation_manager = SimulationManager(parsed_args.client_name)

    time.sleep(0.5)
    controllers = []

    if parsed_args.run_psm_one is True:
        arm_name = 'psm1'
        psm = PSM(simulation_manager, arm_name, tool_id=tool_id)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM1
            # init_xyz = [0.1, -0.85, -0.15]
            init_xyz = [0, 0, -0.10]
            init_rpy = [3.14, 0, 1.57079]
            gui = ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 0.3, 10.0, 0.000001)
            controller = PSMController(gui, psm)
            controllers.append(controller)

    if parsed_args.run_psm_two is True:
        arm_name = 'psm2'
        psm = PSM(simulation_manager, arm_name, tool_id=tool_id)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM2
            init_xyz = [0, 0.0, -0.10]
            init_rpy = [3.14, 0, 1.57079]
            gui = ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 0.3, 12, 0.000001)
            controller = PSMController(gui, psm)
            controllers.append(controller)

    if parsed_args.run_psm_three is True:
        arm_name = 'psm3'
        psm = PSM(simulation_manager, arm_name, tool_id=tool_id)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM2
            init_xyz = [0, 0.0, -0.10]
            init_rpy = [3.14, 0, 1.57079]
            gui = ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 0.3, 3.14, 0.000001)
            controller = PSMController(gui, psm)
            controllers.append(controller)

    if parsed_args.run_ecm is True:
        arm_name = 'CameraFrame'
        ecm = ECM(simulation_manager, arm_name)
        gui = JointGUI('ECM JP', 4, ["ecm j0", "ecm j1", "ecm j2", "ecm j3"], lower_lims=ecm.get_lower_limits(),
                       upper_lims=ecm.get_upper_limits())
        controller = ECMController(gui, ecm)
        controllers.append(controller)

    if len(controllers) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        while not rospy.is_shutdown():
            for cont in controllers:
                cont.run()
            time.sleep(0.005)
