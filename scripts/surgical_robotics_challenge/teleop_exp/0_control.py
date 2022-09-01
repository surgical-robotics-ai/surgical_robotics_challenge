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


#     \author    <hishida3@jhu.edu>
#     \author    Hisashi Ishida
#     \version   1.0
# */
# //==============================================================================

import sys
import time
import numpy as np
from argparse import ArgumentParser

import rospy
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Pose
from PyKDL import Frame, Rotation, Vector, Wrench, Twist
from ambf_client import Client

from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.teleoperation.input_devices.mtm_device_crtk import MTM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI

### Prediction using KalmanFilter ###
from pykalman import KalmanFilter


dt = 0.5 #0.035
motion_scale = 0.04#0.035

# PyKDL types <--> Numpy types
def from_kdl_vector(vector):
    """
    Converts a C{PyKDL.Vector} with fields into a numpy array.
    @type  vector: PyKDL.Vector
    @param vector: The C{PyKDL.Vector} to be converted
    @rtype: np.array
    @return: The resulting numpy array
    """
    return np.array([vector.x(),vector.y(),vector.z()])

def from_kdl_twist(twist):
    """
    Converts a C{PyKDL.Twist} with fields into a numpy array.
    @type  twist: PyKDL.Twist
    @param twist: The C{PyKDL.Twist} to be converted
    @rtype: np.array
    @return: The resulting numpy array
    """
    array = np.zeros(6)
    array[:3] = from_kdl_vector(twist.vel)
    array[3:] = from_kdl_vector(twist.rot)
    return array

def move_shadow_peg(c, peg_list, shadow_list, comloss):
    
    for i in range(6):
        if (not comloss):
            shadow_list[i].set_pose(Pose(peg_list[i].get_pos(), peg_list[i].get_rot()))



class ControllerInterface:
    def __init__(self, leader, psm_arms, camera):
        self.counter = 0
        self.leader = leader

        self.psm_arm = psm_arms[1]
        self.psm_ghost_arm = psm_arms[0]
        

        self.gui = JointGUI('ECM JP', 4, ["ecm j0", "ecm j1", "ecm j2", "ecm j3"])

        self.cmd_xyz = self.psm_arm.T_t_b_home.p
        self.cmd_rpy = None
        self.T_IK = None
        self.T_IK_loss = None
        self._camera = camera

        self._T_c_b = None
        self._update_T_c_b = True

        self.leader.enable_gravity_comp()
        self.leader.enable_orientation_abs()

        self.communication_loss = False
        self.enable_ghost = False


        self.subscribe_communicationLoss()

        # Inititalize your peg
        self.c = Client()
        self.c.connect()
        time.sleep(1.0)
        peg1 = c.get_obj_handle("PuzzleRed1")
        peg2 = c.get_obj_handle("PuzzleRed2")
        peg3 = c.get_obj_handle("PuzzleRed3")
        peg4 = c.get_obj_handle("PuzzleRed4")
        peg5 = c.get_obj_handle("PuzzleRed5")
        peg6 = c.get_obj_handle("PuzzleYellow")

        shadow1 = c.get_obj_handle("One_shadow")
        shadow2 = c.get_obj_handle("Two_shadow")
        shadow3 = c.get_obj_handle("Three_shadow")
        shadow4 = c.get_obj_handle("Four_shadow")
        shadow5 = c.get_obj_handle("Five_shadow")
        shadow6 = c.get_obj_handle("Six_shadow")

        self.peg_list = [peg1, peg2, peg3, peg4, peg5, peg6]
        self.shadow_list = [shadow1, shadow2, shadow3, shadow4, shadow5, shadow6]
        move_shadow_peg(self.c, self.peg_list, self.shadow_list, False)



    def update_T_b_c(self):
        if self._update_T_c_b or self._camera.has_pose_changed:
            self._T_c_b = self.psm_arm.get_T_w_b() * self._camera.get_T_c_w()
            self._update_T_c_b = False

    def update_camera_pose(self):
        self.gui.App.update()
        self._camera.servo_jp(self.gui.jnt_cmds)



    def update_arms_pose_withloss_control(self):
         # update camera pose
        self.update_T_b_c()

        if self.communication_loss:
            self.leader.servo_cp(self.T_IK_loss)

        elif  self.leader.clutch_button_pressed or (self.leader.coag_button_pressed): 
            # send 0 force and torque meaning keep the MTM in the same position
            f = Wrench()
            self.leader.servo_cf(f)

        else:
            if self.leader.is_active() and self.communication_loss == False:
                # Bring back the leader to the previous mtm place
                self.leader.servo_cp(self.leader.pre_coag_pose_msg)
        

        twist = self.leader.measured_cv() * motion_scale#0.007#0.035 ## Vel times dt
        self.cmd_xyz = self.psm_arm.T_t_b_home.p        

        if not self.leader.clutch_button_pressed:
            delta_t = self._T_c_b.M * twist.vel
            self.cmd_xyz = self.cmd_xyz + delta_t
            self.psm_arm.T_t_b_home.p = self.cmd_xyz
        
        if self.leader.coag_button_pressed:

            if (self.communication_loss == False):
                self.cmd_rpy = self._T_c_b.M * self.leader.measured_cp().M
                self.T_IK = Frame(self.cmd_rpy, self.cmd_xyz)

                # self.psm_arm.servo_cp(self.T_IK)
                self.psm_ghost_arm.servo_cp(self.T_IK)
            
                # Move the robot jaw links only if there is a communication
                # self.psm_arm.set_jaw_angle(self.leader.get_jaw_angle())
                self.psm_ghost_arm.set_jaw_angle(self.leader.get_jaw_angle())

                self.T_IK_loss = self.T_IK
            

            # Communication Lost
            else:
                cmd_rpy = self._T_c_b.M * self.leader.measured_cp().M
                self.T_IK = Frame(cmd_rpy, self.cmd_xyz)
                # self.psm_arm.servo_cp(self.T_IK)


    def update_arms_pose_withloss(self):
        self.update_T_b_c()
        if self.leader.coag_button_pressed or self.leader.clutch_button_pressed:
            # self.leader.optimize_wrist_platform()
            f = Wrench()
            self.leader.servo_cf(f)
        else:
            if self.leader.is_active():
                self.leader.servo_cp(self.leader.pre_coag_pose_msg)
        twist = self.leader.measured_cv() * 0.0035
        self.cmd_xyz = self.psm_arm.T_t_b_home.p

        if not self.leader.clutch_button_pressed:
            delta_t = self._T_c_b.M * twist.vel
            self.cmd_xyz = self.cmd_xyz + delta_t
            self.psm_arm.T_t_b_home.p = self.cmd_xyz
            self.psm_ghost_arm.T_t_b_home.p = self.cmd_xyz

        if self.leader.coag_button_pressed:
            self.cmd_rpy = self._T_c_b.M * self.leader.measured_cp().M
            self.T_IK = Frame(self.cmd_rpy, self.cmd_xyz)
            
            if (self.communication_loss == False):
                self.psm_arm.servo_cp(self.T_IK)
                self.psm_ghost_arm.servo_cp(self.T_IK)

        if (self.communication_loss == False):
            self.psm_arm.set_jaw_angle(self.leader.get_jaw_angle())
            self.psm_ghost_arm.set_jaw_angle(self.leader.get_jaw_angle())

    def update_arm_pose(self):
        self.update_T_b_c()
        if self.leader.coag_button_pressed or self.leader.clutch_button_pressed:
            # self.leader.optimize_wrist_platform()
            f = Wrench()
            self.leader.servo_cf(f)
        else:
            if self.leader.is_active():
                self.leader.servo_cp(self.leader.pre_coag_pose_msg)
        twist = self.leader.measured_cv() * 0.0035
        self.cmd_xyz = self.psm_arm.T_t_b_home.p
        if not self.leader.clutch_button_pressed:
            delta_t = self._T_c_b.M * twist.vel
            self.cmd_xyz = self.cmd_xyz + delta_t
            self.psm_arm.T_t_b_home.p = self.cmd_xyz
        if self.leader.coag_button_pressed:
            self.cmd_rpy = self._T_c_b.M * self.leader.measured_cp().M
            self.T_IK = Frame(self.cmd_rpy, self.cmd_xyz)
            self.psm_arm.servo_cp(self.T_IK)
        self.psm_arm.set_jaw_angle(self.leader.get_jaw_angle())

    def communication_loss_callback(self, data):
        self.communication_loss = data.data

    def subscribe_communicationLoss(self):
        rospy.Subscriber("communication_loss", Bool, self.communication_loss_callback)

    def run(self):
        
        # self.update_camera_pose()
        self.update_arms_pose_withloss_control() # with no assistance
        move_shadow_peg(self.c, self.peg_list, self.shadow_list, self.communication_loss)
        # self.update_arm_pose()



if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--mtm', action='store', dest='mtm_name', help='Name of MTM to Bind', default='/dvrk/MTMR/')

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

    run_psm_one = False
    run_psm_two = False

    print(parsed_args.mtm_name)

    if (parsed_args.mtm_name == '/MTMR/' or parsed_args.mtm_name =='/dvrk/MTMR'):
        print("Loading the mtmr....")
        c = Client('mtmr')
        c.connect()
        run_psm_one = False
        run_psm_two = True

    if (parsed_args.mtm_name == '/MTML/' or parsed_args.mtm_name =='/dvrk/MTML'):
        print("Loading the mtml....")
        c = Client('mtml')
        c.connect()
        run_psm_one = True
        run_psm_two = False

    cam = ECM(c, 'CameraFrame')
    time.sleep(0.5)

    controllers = []
    psm_arms = []

    if run_psm_one is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]

        arm_name = 'psm1'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(-0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)

        arm_name = 'psm1_ghost'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(-0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)


    if run_psm_two is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm2'
        print('LOADING CONTROLLER FOR ', arm_name)
        theta_base = -0.7
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)
        
        arm_name = 'psm2_ghost'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)
        
        

    if len(psm_arms) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:

        leader = MTM(parsed_args.mtm_name)
        leader.set_base_frame(Frame(Rotation.RPY((3.14 - 0.8) / 2, 0, 0), Vector(0, 0, 0)))
        controller1 = ControllerInterface(leader, psm_arms, cam)
        controllers.append(controller1)
        rate = rospy.Rate(120)


        tick_pub = rospy.Publisher(parsed_args.mtm_name + 'tick', Empty, queue_size=1)
        while not rospy.is_shutdown():
            for cont in controllers:
                cont.run()
            tick_pub.publish(Empty())
            rate.sleep()
        # i_count = 0
        # start = time.time()
        # while i_count < 5000:
        #     for cont in controllers:
        #         cont.run()
        #     rate.sleep()
        #     i_count+=1
        # end = time.time()
        # print("elapsed time:", end -start)
        # print("time per loop;", (end-start)/5000)

        if rospy.is_shutdown():
            print("shutdowning...")

