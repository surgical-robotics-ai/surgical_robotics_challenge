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
from std_msgs.msg import Bool
from PyKDL import Frame, Rotation, Vector, Wrench, Twist
from ambf_client import Client

from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.teleoperation.input_devices.mtm_device_crtk import MTM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI

### Prediction using KalmanFilter ###
from pykalman import KalmanFilter


dt = 0.5 #0.035

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


class KFPredict:
    def __init__(self, observation):
        
        self.transition_matrix = [
            [1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0, 0],
            [0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0],
            [0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt],
            [0, 0, 0, 1, 0, 0, dt, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, dt, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, dt],
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1],
        ]
        observation_matrix = [
            [1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1],
        ]
        initial_state_covariance = 0.01 * np.ones([9, 9])
        initial_observation_covariance = 0.01 * np.ones([9, 9])

        self.kf = KalmanFilter(
            transition_matrices=self.transition_matrix,
            observation_matrices=observation_matrix,
            initial_state_mean=observation,
            initial_state_covariance=initial_state_covariance,
            observation_covariance=initial_observation_covariance,
        )
    
    def predict(self, state, cov):
        state = state.reshape([9,])
        mean, covariance = self.kf.filter_update(
            filtered_state_mean=state,
            filtered_state_covariance=cov,
            observation=self.transition_matrix @ state, 
        )
        test_observation_covariance = 0.1 * np.ones([9,9])
        #covariance = covariance + (1.0 - np.exp(-(t - t_change) / 10.0)) * test_observation_covariance
        covariance = covariance + (1.0 - np.exp(-(2.0) / 10.0)) * test_observation_covariance

        return mean, covariance

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
        self.T_IK_predict = None
        self._camera = camera

        self._T_c_b = None
        self._update_T_c_b = True

        self.leader.enable_gravity_comp()

        self.communication_loss = False
        self.enable_ghost = False

        self.vel_prev = Twist.Zero()
        self.observation = np.zeros([1,9])
        self.kf = KFPredict(self.observation)
        self.predict_xyz =self.cmd_xyz


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

        if  self.leader.clutch_button_pressed or (self.leader.coag_button_pressed and self.communication_loss == False): 
            # send 0 force and torque meaning keep the MTM in the same position
            f = Wrench()
            self.leader.servo_cf(f)

        else:
            if self.leader.is_active() and self.communication_loss == False:
                # Bring back the leader to the previous mtm place
                self.leader.servo_cp(self.leader.pre_coag_pose_msg)
        

        twist = self.leader.measured_cv() * 0.035 ## Vel times dt
        self.cmd_xyz = self.psm_arm.T_t_b_home.p

        # acc = Twist.Zero()
        acc = twist - self.vel_prev
        self.vel_prev = twist
        

        if not self.leader.clutch_button_pressed:
            delta_t = self._T_c_b.M * twist.vel
            self.cmd_xyz = self.cmd_xyz + delta_t
            self.psm_arm.T_t_b_home.p = self.cmd_xyz
        
        if self.leader.coag_button_pressed:

            if (self.communication_loss == False):
                self.cmd_rpy = self._T_c_b.M * self.leader.measured_cp().M
                self.T_IK = Frame(self.cmd_rpy, self.cmd_xyz)

                pos = np.array([self.cmd_xyz[0],self.cmd_xyz[1],self.cmd_xyz[2]])
                vel = from_kdl_twist(twist)
                acc_np = from_kdl_twist(acc)
                self.observation = np.hstack([pos,vel[:3],acc_np[:3]])

                self.psm_arm.servo_cp(self.T_IK)
                self.psm_ghost_arm.servo_cp(self.T_IK)
            
                # Move the robot jaw links only if there is a communication
                self.psm_arm.set_jaw_angle(self.leader.get_jaw_angle())
                self.psm_ghost_arm.set_jaw_angle(self.leader.get_jaw_angle())
            

            # Communication Lost
            else:
                cmd_rpy = self._T_c_b.M * self.leader.measured_cp().M
                self.T_IK = Frame(cmd_rpy, self.cmd_xyz)
                self.psm_arm.servo_cp(self.T_IK)


    def update_arms_pose_withloss(self):
        self.update_T_b_c()
        if self.leader.coag_button_pressed or self.leader.clutch_button_pressed:
            # self.leader.optimize_wrist_platform()
            f = Wrench()
            self.leader.servo_cf(f)
        else:
            if self.leader.is_active():
                self.leader.servo_cp(self.leader.pre_coag_pose_msg)
        twist = self.leader.measured_cv() * 0.035
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

    def communication_loss_callback(self, data):
        self.communication_loss = data.data

    def subscribe_communicationLoss(self):
        rospy.Subscriber("communication_loss", Bool, self.communication_loss_callback)

    def run(self):
        
        self.update_camera_pose()

        self.update_arms_pose_withloss_control() # with no assistance
        self.subscribe_communicationLoss()



if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='ambf_client')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
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

    if parsed_args.run_psm_one in ['True', 'true', '1']:
        parsed_args.run_psm_one = True
    elif parsed_args.run_psm_one in ['False', 'false', '0']:
        parsed_args.run_psm_one = False

    if parsed_args.run_psm_two in ['True', 'true', '1']:
        parsed_args.run_psm_two = True
    elif parsed_args.run_psm_two in ['False', 'false', '0']:
        parsed_args.run_psm_two = False

    c = Client(parsed_args.client_name)
    c.connect()

    cam = ECM(c, 'CameraFrame')
    time.sleep(0.5)

    controllers = []
    psm_arms = []

    if parsed_args.run_psm_one is True:
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


    if parsed_args.run_psm_two is True:
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
        rate = rospy.Rate(200)

        while not rospy.is_shutdown():
            for cont in controllers:
                cont.run()
            rate.sleep()

        if rospy.is_shutdown():
            print("shutdowning...")

