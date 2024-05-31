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


#     \author    <amunawar@jhu.edu>, <hzhou6@wpi.edu>
#     \author    Adnan Munawar, Haoying(Jack) Zhou
#     \version   1.0
# */
# //==============================================================================

import numpy as np
from surgical_robotics_challenge.utils.utilities import *
from surgical_robotics_challenge.kinematics.DH import *
import os
import sys
dynamic_path = os.path.abspath(__file__ + "/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from glob import glob
import json

config_folder = os.path.join(dynamic_path, 'kinematics', 'config')

# THIS IS THE FK FOR THE PSM MOUNTED WITH THE LARGE NEEDLE DRIVER TOOL. THIS IS THE
# SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL. NOTE, JUST LIKE A FAULT IN THE
# MTM's DH PARAMETERS IN THE MANUAL, THERE IS A FAULT IN THE PSM's DH AS WELL. BASED ON
# THE FRAME ATTACHMENT IN THE DVRK MANUAL THE CORRECT DH CAN FOUND IN THIS FILE

# ALSO, NOTICE THAT AT HOME CONFIGURATION THE TIP OF THE PSM HAS THE FOLLOWING
# ROTATION OFFSET W.R.T THE BASE. THIS IS IMPORTANT FOR IK PURPOSES.
# R_7_0 = [ 0,  1,  0 ]
#       = [ 1,  0,  0 ]
#       = [ 0,  0, -1 ]
# Basically, x_7 is along y_0, y_7 is along x_0 and z_7 is along -z_0.

# You need to provide a list of joint positions. If the list is less that the number of joint
# i.e. the robot has 6 joints, but only provide 3 joints. The FK till the 3+1 link will be provided


# Currently implemented PSMs
class PSMType:
    LND = 400006
    LND_SI = 420006
    Default = LND


# Currently implemented Tools
class ToolType:
    LND = 400006
    LND_SI = 420006
    Default = LND


class PSMKinematicSolver:
    def __init__(self, root_dir=config_folder, psm_type=PSMType.Default, tool_id=ToolType.Default):
        self.num_links = 7
        assert root_dir is not None, 'root dir must be provided'
        assert psm_type is not None, 'psm type must be provided'
        assert type(tool_id) == int, "tool id must be an integer, please check the tool id input"
        self.root_dir = root_dir
        self.psm_type = psm_type
        self.tool_id = tool_id
        self.L_rcc = None
        self.L_tool = None
        self.L_pitch2yaw = None
        self.L_yaw2ctrlpnt = 0.0  # Fixed length from the tool yaw joint to the end effector tip
        self.L_tool2rcm_offset = None
        self.kinematics = []
        self.lower_limits = []
        self.upper_limits = []

        self.load_json_files()
        # if tool_id == 400006:
        #     self.L_rcc = 0.4389  # From dVRK documentation
        #     self.L_tool = 0.416  # From dVRK documentation
        #     self.L_pitch2yaw = 0.009  # Fixed length from the palm joint to the pinch joint
        #     self.L_yaw2ctrlpnt = 0.0  # Fixed length from the pinch joint to the pinch tip
        #     self.L_tool2rcm_offset = 0.0229 # Distance between tool tip and the Remote Center of Motion at Home Pose
        # elif tool_id == 420006:
        #     self.L_rcc = 0.4318  # From dVRK documentation, keep the same as the classical tool
        #     self.L_tool = 0.4826  # L_rcc - L_tool2rcm_offset
        #     self.L_pitch2yaw = 0.009  # Fixed length from the tool pitch link to the tool yaw link
        #     self.L_yaw2ctrlpnt = 0.0  # Fixed length from the tool yaw link to the tool tip
        #     self.L_tool2rcm_offset = -0.0508  # Distance between tool pitch link and the Remote Center of Motion at Home Pose
        # else:
        #     raise ValueError('Invalid tool_id')
        # PSM DH Params
        # alpha | a | theta | d | offset | type
        # self.kinematics_test = [DH(PI_2, 0, 0, 0, PI_2, JointType.REVOLUTE, Convention.MODIFIED),
        #                    DH(-PI_2, 0, 0, 0, -PI_2,
        #                       JointType.REVOLUTE, Convention.MODIFIED),
        #                    DH(PI_2, 0, 0, 0, -self.L_rcc,
        #                       JointType.PRISMATIC, Convention.MODIFIED),
        #                    DH(0, 0, 0, self.L_tool, 0,
        #                       JointType.REVOLUTE, Convention.MODIFIED),
        #                    DH(-PI_2, 0, 0, 0, -PI_2,
        #                       JointType.REVOLUTE, Convention.MODIFIED),
        #                    DH(-PI_2, self.L_pitch2yaw, 0, 0, -PI_2,
        #                       JointType.REVOLUTE, Convention.MODIFIED),
        #                    DH(-PI_2, 0, 0, self.L_yaw2ctrlpnt, PI_2, JointType.REVOLUTE, Convention.MODIFIED)]
        #
        # self.lower_limits = [np.deg2rad(-91.96), np.deg2rad(-60), -0.0, np.deg2rad(-175), np.deg2rad(-90), np.deg2rad(-85)]
        #
        # self.upper_limits = [np.deg2rad(91.96), np.deg2rad(60), 0.240, np.deg2rad(175), np.deg2rad(90), np.deg2rad(85)]

    @staticmethod
    def load_convention_type(convention_type: str):
        if convention_type == 'modified':
            return Convention.MODIFIED
        else:
            return Convention.STANDARD

    @staticmethod
    def load_joint_type(joint_type: str):
        if joint_type == 'revolute':
            return JointType.REVOLUTE
        elif joint_type == 'prismatic':
            return JointType.PRISMATIC
        else:
            raise ValueError('incorrect joint type')

    def load_json_files(self):
        psm_file_path = os.path.join(self.root_dir, 'kinematic', f'psm_{str(self.psm_type)}.json')
        tool_file_list = glob(os.path.join(self.root_dir, 'tool', f'*{str(self.tool_id)}.json'))
        assert len(tool_file_list) == 1, 'multiple tool files, please check the json configuration files'
        tool_file_path = tool_file_list[0]
        psm_obj = load_json_dvrk(psm_file_path)
        tool_obj = load_json_dvrk(tool_file_path)
        self.L_rcc = -psm_obj['DH']['joints'][2]['offset']
        self.L_tool = tool_obj['DH']['joints'][0]['D']
        self.L_pitch2yaw = tool_obj['DH']['joints'][2]['A']
        self.L_tool2rcm_offset = self.L_rcc - self.L_tool

        for i_obj in [psm_obj, tool_obj]:
            dict_DH = i_obj['DH']
            obj_convention = self.load_convention_type(dict_DH['convention'])
            list_joints = dict_DH['joints']
            for i_joint in range(3):
                dict_joint = list_joints[i_joint]
                link_DH = DH(dict_joint['alpha'], dict_joint['A'], dict_joint['theta'], dict_joint['D'],
                             dict_joint['offset'], self.load_joint_type(dict_joint['type']), obj_convention)
                self.kinematics.append(link_DH)
                self.lower_limits.append(dict_joint['qmin'])
                self.upper_limits.append(dict_joint['qmax'])

        last_link = DH(-np.pi / 2, 0, 0, self.L_yaw2ctrlpnt, np.pi / 2, JointType.REVOLUTE, Convention.MODIFIED)
        self.kinematics.append(last_link)
        print('kinematics loaded from', psm_file_path, ' and ', tool_file_path )

    def get_link_params(self, link_num):
        if link_num < 0 or link_num > self.num_links:
            # Error
            print("ERROR, ONLY ", self.num_links, " JOINT DEFINED")
            return []
        else:
            return self.kinematics[link_num]

    def compute_FK(self, joint_pos:list, up_to_link:int)->np.matrix:
        '''
        Compute the forward kinematic matrix
        :param file_path: joint value list
        :param up_to_link: number of links
        :return: the 4x4 transformation matrix from psm base to psm end effector
        '''
        if up_to_link > self.num_links:
            raise "ERROR! COMPUTE FK UP_TO_LINK GREATER THAN DOF"
        j = [0, 0, 0, 0, 0, 0, 0]
        for i in range(len(joint_pos)):
            j[i] = joint_pos[i]

        T_N_0 = np.identity(4)

        for i in range(up_to_link):
            link_dh = self.get_link_params(i)
            link_dh.theta = j[i]
            T_N_0 = T_N_0 * link_dh.get_trans()

        return T_N_0

    def compute_IK(self, T_7_0):
        # Pinch Joint
        T_PinchJoint_7 = Frame(Rotation.RPY(0, 0, 0),
                               self.L_yaw2ctrlpnt * Vector(0.0, 0.0, -1.0))
        # Pinch Joint in Origin
        T_PinchJoint_0 = T_7_0 * T_PinchJoint_7

        # It appears from the geometry of the robot, that the palm joint is always in the ZY
        # plane of the end effector frame (7th Frame)
        # This is the logic that should give us the direction of the palm link and then
        # we know the length of the palm link so we can keep going back to find the shaftTip (PalmJoint)
        # position

        # Convert the vector from base to pinch joint in the pinch joint frame
        # print("P_PinchJoint_0: ", round_vec(T_PinchJoint_0.p))
        R_0_PinchJoint = T_PinchJoint_0.M.Inverse()
        P_PinchJoint_local = R_0_PinchJoint * T_PinchJoint_0.p
        # print("P_PinchJoint_local: ", round_vec(P_PinchJoint_local))
        # Now we can trim the value along the x axis to get a projection along the YZ plane as mentioned above
        N_PalmJoint_PinchJoint = -P_PinchJoint_local
        N_PalmJoint_PinchJoint[0] = 0
        N_PalmJoint_PinchJoint.Normalize()

        # We can check the angle to see if things make sense
        # angle = get_angle(N_PalmJoint_PinchJoint, Vector(0, 0, -1))
        # print("Palm Link Angle in Pinch YZ Plane: ", angle)

        # # If the angle between the two vectors is > 90 Degree, we should move in the opposite direction
        # if angle > np.pi/2:
        #     N_PalmJoint_PinchJoint = N_PalmJoint_PinchJoint
        #
        # print(angle)

        # Add another frame to account for Palm link length
        T_PalmJoint_PinchJoint = Frame(Rotation.RPY(0, 0, 0),
                                       N_PalmJoint_PinchJoint * self.L_pitch2yaw)
        # Get the shaft tip or the Palm's Joint position
        T_PalmJoint_0 = T_7_0 * T_PinchJoint_7 * T_PalmJoint_PinchJoint

        # Calculate insertion_depth to check if the tool is past the RCM
        insertion_depth = T_PalmJoint_0.p.Norm()

        # Now having the end point of the shaft or the PalmJoint, we can calculate some
        # angles as follows
        xz_diagonal = math.sqrt(T_PalmJoint_0.p[0] ** 2 + T_PalmJoint_0.p[2] ** 2)
        # # print('XZ Diagonal: ', xz_diagonal)

        j1 = math.atan2(T_PalmJoint_0.p[0], -T_PalmJoint_0.p[2])

        j2 = -math.atan2(T_PalmJoint_0.p[1], xz_diagonal)

        j3 = insertion_depth + self.L_tool2rcm_offset

        # Calculate j4
        # This is an important case and has to be dealt carefully. Based on some inspection, we can find that
        # we need to construct a plane based on the vectors Rx_7_0 and D_PinchJoint_PalmJoint_0 since these are
        # the only two vectors that are orthogonal at all configurations of the EE.
        cross_palmlink_x7_0 = T_7_0.M.UnitX() * (T_PinchJoint_0.p - T_PalmJoint_0.p)

        # To get j4, compare the above vector with Y axes of T_3_0
        T_3_0 = convert_mat_to_frame(self.compute_FK([j1, j2, j3], 3))
        j4 = get_angle(cross_palmlink_x7_0, T_3_0.M.UnitY(), up_vector=-T_3_0.M.UnitZ())

        # Calculate j5
        # This should be simple, just compute the angle between Rz_4_0 and D_PinchJoint_PalmJoint_0
        link4_dh = self.get_link_params(3)
        link4_dh.theta = j4
        T_4_3 = convert_mat_to_frame(link4_dh.get_trans())
        T_4_0 = T_3_0 * T_4_3

        j5 = get_angle(T_PinchJoint_0.p - T_PalmJoint_0.p, T_4_0.M.UnitZ(), up_vector=-T_4_0.M.UnitY())

        # Calculate j6
        # This too should be simple, compute the angle between the Rz_7_0 and Rx_5_0.
        link5_dh = self.get_link_params(4)
        link5_dh.theta = j5
        T_5_4 = convert_mat_to_frame(link5_dh.get_trans())
        T_5_0 = T_4_0 * T_5_4

        j6 = get_angle(T_7_0.M.UnitZ(), T_5_0.M.UnitX(), up_vector=-T_5_0.M.UnitY())

        return [j1, j2, j3, j4, j5, j6]


if __name__ == "__main__":
    file_folder = os.path.join(dynamic_path, 'kinematics', 'config')
    psm_type = 420006
    tool_id = 420006

    psm_ks = PSMKinematicSolver(psm_type=psm_type, tool_id=tool_id)

    joint_list_ref = [0.1, 0.1, 0.2, 0.1, 0.1, 0.1]
    T_7_0 = psm_ks.compute_FK(joint_list_ref, 7)
    print(T_7_0)
    T_7_0 = convert_mat_to_frame(T_7_0)

    calculated_joint_list = psm_ks.compute_IK(T_7_0)
    print('Requested Joint: \n', joint_list_ref)
    print('Calculated Joint: \n', calculated_joint_list)

    psm_file_path = os.path.join(file_folder, 'kinematic', f'psm_{str(psm_type)}.json')
    tool_file_list = glob(os.path.join(file_folder, 'tool', f'*{str(tool_id)}.json'))
    tool_file_path = tool_file_list[0]
    psm_obj = load_json_dvrk(psm_file_path)
    tool_obj = load_json_dvrk(tool_file_path)
