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
import sys
from surgical_robotics_challenge.simulation_manager import SimulationManager
import time
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from surgical_robotics_challenge.teleoperation.input_devices.mtm_device_crtk import MTM
from itertools import cycle
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from surgical_robotics_challenge.utils.utilities import get_boolean_from_opt
from surgical_robotics_challenge.teleoperation.mtm_teleop_common import apply_mtm_to_psm_command, load_selected_psm_arms


class ControllerInterface:
    def __init__(self, leader, psm_arms, ecm, update_frequency, enable_force_feedback=False):
        self.counter = 0
        self.leader = leader
        self.psm_arms = cycle(psm_arms)
        if sys.version_info[0] >= 3:
            self.active_psm = next(self.psm_arms)
        else:
            self.active_psm = self.psm_arms.next()
        self.gui = JointGUI('ECM JP', 4, ["ecm j0", "ecm j1", "ecm j2", "ecm j3"], lower_lims=cam.get_lower_limits(),
                            upper_lims=cam.get_upper_limits())

        self.cmd_xyz = self.active_psm.T_t_b_home.p
        self.cmd_rpy = None
        self.T_IK = None
        self._ecm = ecm
        self.update_dt = 1.0 / update_frequency

        self._T_c_b = None
        self._update_T_c_b = True
        self._enable_force_feedback = enable_force_feedback

        self.leader.enable_gravity_comp()

    def switch_psm(self):
        self._update_T_c_b = True
        if sys.version_info[0] >= 3:
            self.active_psm = next(self.psm_arms)
        else:
            self.active_psm = self.psm_arms.next()
        print('Switching Control of Next PSM Arm: ', self.active_psm.name)

    def update_T_b_c(self):
        if self._update_T_c_b or self._ecm.has_pose_changed():
            self._T_c_b = self.active_psm.get_T_w_b() * self._ecm.get_T_c_w()
            self._update_T_c_b = False

    def update_camera_pose(self):
        self.gui.App.update()
        self._ecm.servo_jp(self.gui.jnt_cmds)

    def update_arm_pose(self):
        self.update_T_b_c()
        self.cmd_xyz, self.T_IK = apply_mtm_to_psm_command(
            self.leader,
            self.active_psm,
            self._T_c_b,
            self.update_dt,
            set_jaw_only_when_coag=False,
            enable_force_feedback=self._enable_force_feedback,
        )

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.active_psm.target_IK is not None:
            T_t_w = self.active_psm.get_T_b_w() * self.T_IK
            self.active_psm.target_IK.set_pose(T_t_w)
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
        if self.leader.switch_psm:
            self.switch_psm()
            self.leader.switch_psm = False
        self.update_camera_pose()
        self.update_arm_pose()
        # self.update_visual_markers()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='mtm_sim_teleop')
    parser.add_argument('-t', action='store', dest='tool_id', help='Surgical Instrument Serial Number', default='400006')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=False)
    parser.add_argument('--mtm', action='store', dest='mtm_name', help='Name of MTM to Bind', default='/dvrk/MTMR/')
    parser.add_argument('--update_frequency', action='store', dest='update_frequency', help='Update Frequency', default=200)
    parser.add_argument('-e', '--enable_force_feedback', action='store', dest='enable_force_feedback', help='Enable MTM force feedback', default=False)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    mtm_valid_list = ['/MTMR/, /MTML/', '/dvrk/MTMR/', '/dvrk/MTML/', 'MTMR', 'MTML']
    if parsed_args.mtm_name in mtm_valid_list:
        if parsed_args.mtm_name in ['MTMR', 'MTML']:
            parsed_args.mtm_name = '/' + parsed_args.mtm_name + '/'
    else:
        print('ERROR! --mtm argument should be one of the following', mtm_valid_list)
        raise ValueError

    parsed_args.run_psm_one = get_boolean_from_opt(parsed_args.run_psm_one)
    parsed_args.run_psm_two = get_boolean_from_opt(parsed_args.run_psm_two)
    parsed_args.run_psm_three = get_boolean_from_opt(parsed_args.run_psm_three)
    parsed_args.enable_force_feedback = get_boolean_from_opt(parsed_args.enable_force_feedback)

    simulation_manager = SimulationManager(parsed_args.client_name)

    cam = ECM(simulation_manager, 'CameraFrame')
    time.sleep(0.5)

    controllers = []

    tool_id = int(parsed_args.tool_id)

    psm_by_name = load_selected_psm_arms(
        simulation_manager,
        cam,
        parsed_args.run_psm_one,
        parsed_args.run_psm_two,
        parsed_args.run_psm_three,
        tool_id=tool_id,
    )
    psm_arms = list(psm_by_name.values())

    if len(psm_arms) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        leader = MTM(simulation_manager.get_ral(), parsed_args.mtm_name)
        leader.set_base_frame(Frame(Rotation.RPY((3.14 - 0.8) / 2, 0, 0), Vector(0, 0, 0)))
        controller1 = ControllerInterface(
            leader,
            psm_arms,
            cam,
            update_frequency=int(parsed_args.update_frequency),
            enable_force_feedback=parsed_args.enable_force_feedback,
        )
        controllers.append(controller1)

        rate = simulation_manager.get_ral().create_rate(int(parsed_args.update_frequency))
        
        while not simulation_manager.is_shutdown():
            try:
                for cont in controllers:
                    cont.run()
                rate.sleep()
            except Exception as e:
                print(e)
                print('Goodbye')
                break
            