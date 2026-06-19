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
from surgical_robotics_challenge.simulation_manager import SimulationManager
import time
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from surgical_robotics_challenge.teleoperation.input_devices.mtm_device_crtk import MTM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from surgical_robotics_challenge.utils.utilities import get_boolean_from_opt
from surgical_robotics_challenge.teleoperation.mtm_teleop_common import apply_mtm_to_psm_command, load_selected_psm_arms
from threading import Thread
from std_msgs.msg import Float64MultiArray


class ControllerInterface:
    def __init__(
        self,
        ral,
        leader_l,
        leader_r,
        psm_arm_l,
        psm_arm_r,
        ecm,
        update_frequency,
        enable_force_feedback=False,
    ):
        self.counter = 0
        self.leader_1 = leader_l
        self.leader_2 = leader_r
        self.psm_1 = psm_arm_l
        self.psm_2 = psm_arm_r
        self.gui = JointGUI('ECM JP', 4, ["ecm j0", "ecm j1", "ecm j2", "ecm j3"])
        self.update_dt = 1.0 / update_frequency

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
        self._enable_force_feedback = enable_force_feedback
        self._pub_ecm = ral.publisher('/ecm/setpoint_js', Float64MultiArray, queue_size=1)
        self.leader_1.enable_gravity_comp()
        self.leader_2.enable_gravity_comp()

    # def switch_psm(self):
    #     self._update_T_c_b = True
    #     self.active_psm = self.psm_arms.next()
    #     print('Switching Control of Next PSM Arm: ', self.active_psm.name)

    def update_T_b_c(self):
        if self._update_T_c_b or self._ecm.has_pose_changed():
            self._T1_c_b = self.psm_1.get_T_w_b() * self._ecm.get_T_c_w()
            self._T2_c_b = self.psm_2.get_T_w_b() * self._ecm.get_T_c_w()
            self._update_T_c_b = False

    def update_camera_pose(self):
        self.gui.App.update()
        self._ecm.servo_jp(self.gui.jnt_cmds)

    def _teleop_arm(self, leader, psm, T_c_b, idx):
        cmd_xyz, T_ik = apply_mtm_to_psm_command(
            leader,
            psm,
            T_c_b,
            self.update_dt,
            set_jaw_only_when_coag=False,
            enable_force_feedback=self._enable_force_feedback,
        )
        if idx == 1:
            self.cmd1_xyz = cmd_xyz
            self.T1_IK = T_ik
        else:
            self.cmd2_xyz = cmd_xyz
            self.T2_IK = T_ik

    def teleop_pair_1(self):
        self._teleop_arm(self.leader_1, self.psm_1, self._T1_c_b, idx=1)

    def teleop_pair_2(self):
        self._teleop_arm(self.leader_2, self.psm_2, self._T2_c_b, idx=2)

    def update_arm_pose(self):
        self.update_T_b_c()
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
        # if self.leader.switch_psm:
        #     self.switch_psm()
        #     self.leader.switch_psm = False
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
    parser.add_argument('--update_frequency', action='store', dest='update_frequency', help='Update Frequency', default=200)
    parser.add_argument('-e', '--enable_force_feedback', action='store', dest='enable_force_feedback', help='Enable MTM force feedback', default=False)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    parsed_args.run_psm_one = get_boolean_from_opt(parsed_args.run_psm_one)
    parsed_args.run_psm_two = get_boolean_from_opt(parsed_args.run_psm_two)
    parsed_args.run_psm_three = get_boolean_from_opt(parsed_args.run_psm_three)
    parsed_args.enable_force_feedback = get_boolean_from_opt(parsed_args.enable_force_feedback)

    simulation_manager = SimulationManager(parsed_args.client_name)

    # tool_id = int(parsed_args.tool_id)

    cam = ECM(simulation_manager, 'CameraFrame')
    time.sleep(0.5)

    controllers = []

    psm_by_name = load_selected_psm_arms(
        simulation_manager,
        cam,
        parsed_args.run_psm_one,
        parsed_args.run_psm_two,
        parsed_args.run_psm_three,
    )

    psm1 = psm_by_name.get('psm1')
    psm2 = psm_by_name.get('psm2')
    if psm1 is None or psm2 is None:
        print('No Valid PSM Arms Specified')
        print('Pair teleoperation requires both psm1 and psm2')
        print('Exiting')

    else:
        leader_l = MTM(simulation_manager.get_ral(), '/MTML/')
        leader_r = MTM(simulation_manager.get_ral(), '/MTMR/')
        leader_l.set_base_frame(Frame(Rotation.RPY((3.14 - 0.8) / 2, 0, 0), Vector(0, 0, 0)))
        leader_r.set_base_frame(Frame(Rotation.RPY((3.14 - 0.8) / 2, 0, 0), Vector(0, 0, 0)))
        controller1 = ControllerInterface(
            simulation_manager.get_ral(),
            leader_l,
            leader_r,
            psm1,
            psm2,
            cam,
            update_frequency=int(parsed_args.update_frequency),
            enable_force_feedback=parsed_args.enable_force_feedback,
        )
        controllers.append(controller1)

        rate = simulation_manager.create_rate(int(parsed_args.update_frequency))

        while not simulation_manager.is_shutdown():
            try:
                for cont in controllers:
                    cont.run()
                rate.sleep()
            except Exception as e:
                print(e)
                print('Goodbye')
                break
