import jsonpickle
import numpy as np
from surgical_robotics_challenge.utils.utilities import *
from surgical_robotics_challenge.kinematics.DH import *
import os
import sys
dynamic_path = os.path.abspath(__file__ + "/../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from glob import glob
import json
import math
from PyKDL import Vector, Rotation, Frame, dot


class PSMKinematicSolver:
    def __init__(self, root_dir=dynamic_path, psm_type='classic', tool_id=None):
        self.num_links = 7
        self.tool_id = tool_id

        assert type(self.tool_id) == int, "tool id must be an integer, please check the input"

        if tool_id == 400006:
            self.L_rcc = 0.4389  # From dVRK documentation
            self.L_tool = 0.416  # From dVRK documentation
            self.L_pitch2yaw = 0.009  # Fixed length from the palm joint to the pinch joint
            self.L_yaw2ctrlpnt = 0.0  # Fixed length from the pinch joint to the pinch tip
            self.L_tool2rcm_offset = 0.0229 # Distance between tool tip and the Remote Center of Motion at Home Pose
        elif tool_id == 420006:
            self.L_rcc = 0.4318  # From dVRK documentation, keep the same as the classical tool
            self.L_tool = 0.4826  # L_rcc - L_tool2rcm_offset
            self.L_pitch2yaw = 0.009  # Fixed length from the tool pitch link to the tool yaw link
            self.L_yaw2ctrlpnt = 0.0  # Fixed length from the tool yaw link to the tool tip
            self.L_tool2rcm_offset = -0.0508  # Distance between tool pitch link and the Remote Center of Motion at Home Pose
        else:
            raise ValueError('Invalid tool_id')
        # PSM DH Params
        # alpha | a | theta | d | offset | type
        self.kinematics = [DH(PI_2, 0, 0, 0, PI_2, JointType.REVOLUTE, Convention.MODIFIED),
                           DH(-PI_2, 0, 0, 0, -PI_2,
                              JointType.REVOLUTE, Convention.MODIFIED),
                           DH(PI_2, 0, 0, 0, -self.L_rcc,
                              JointType.PRISMATIC, Convention.MODIFIED),
                           DH(0, 0, 0, self.L_tool, 0,
                              JointType.REVOLUTE, Convention.MODIFIED),
                           DH(-PI_2, 0, 0, 0, -PI_2,
                              JointType.REVOLUTE, Convention.MODIFIED),
                           DH(-PI_2, self.L_pitch2yaw, 0, 0, -PI_2,
                              JointType.REVOLUTE, Convention.MODIFIED),
                           DH(-PI_2, 0, 0, self.L_yaw2ctrlpnt, PI_2, JointType.REVOLUTE, Convention.MODIFIED)]

        self.lower_limits = [np.deg2rad(-91.96), np.deg2rad(-60), -0.0, np.deg2rad(-175), np.deg2rad(-90), np.deg2rad(-85)]

        self.upper_limits = [np.deg2rad(91.96), np.deg2rad(60), 0.240, np.deg2rad(175), np.deg2rad(90), np.deg2rad(85)]

    def get_link_params(self, link_num):
        if link_num < 0 or link_num > self.num_links:
            # Error
            print("ERROR, ONLY ", self.num_links, " JOINT DEFINED")
            return []
        else:
            return self.kinematics[link_num]


    def compute_FK(self, joint_pos, up_to_link):
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
        # print("N_PalmJoint_PinchJoint: ", round_vec(N_PalmJoint_PinchJoint))
        T_PalmJoint_PinchJoint = Frame(Rotation.RPY(
            0, 0, 0), N_PalmJoint_PinchJoint * self.L_pitch2yaw)
        # print("P_PalmJoint_PinchJoint: ", round_vec(T_PalmJoint_PinchJoint.p))
        # Get the shaft tip or the Palm's Joint position
        T_PalmJoint_0 = T_7_0 * T_PinchJoint_7 * T_PalmJoint_PinchJoint

        # print("P_PalmJoint_0: ", round_vec(T_PalmJoint_0.p))
        # print("P_PinchJoint_0: ", round_vec(T_PinchJoint_0.p))
        # Now this should be the position of the point along the RC
        # print("Point Along the SHAFT: ", T_PalmJoint_0.p)

        # Calculate insertion_depth to check if the tool is past the RCM
        insertion_depth = T_PalmJoint_0.p.Norm()

        # Now having the end point of the shaft or the PalmJoint, we can calculate some
        # angles as follows
        xz_diagonal = math.sqrt(T_PalmJoint_0.p[0] ** 2 + T_PalmJoint_0.p[2] ** 2)
        # # print('XZ Diagonal: ', xz_diagonal)

        # yz_diagonal = math.sqrt(T_PalmJoint_0.p[1] ** 2 + T_PalmJoint_0.p[2] ** 2)
        # # print('YZ Diagonal: ', yz_diagonal)

        j1 = math.atan2(T_PalmJoint_0.p[0], -T_PalmJoint_0.p[2])

        # j2 = np.sign(T_PalmJoint_0.p[0]) * math.acos(-T_PalmJoint_0.p[2] / yz_diagonal)
        j2 = -math.atan2(T_PalmJoint_0.p[1], xz_diagonal)

        j3 = insertion_depth + self.L_tool2rcm_offset

        # Calculate j4
        # This is an important case and has to be dealt carefully. Based on some inspection, we can find that
        # we need to construct a plane based on the vectors Rx_7_0 and D_PinchJoint_PalmJoint_0 since these are
        # the only two vectors that are orthogonal at all configurations of the EE.
        cross_palmlink_x7_0 = T_7_0.M.UnitX() * (T_PinchJoint_0.p - T_PalmJoint_0.p)

        # To get j4, compare the above vector with Y axes of T_3_0
        T_3_0 = convert_mat_to_frame(self.compute_FK([j1, j2, j3], 3))
        j4 = get_angle(cross_palmlink_x7_0, T_3_0.M.UnitY(),
                       up_vector=-T_3_0.M.UnitZ())

        # Calculate j5
        # This should be simple, just compute the angle between Rz_4_0 and D_PinchJoint_PalmJoint_0
        link4_dh = self.get_link_params(3)
        link4_dh.theta = j4
        T_4_3 = convert_mat_to_frame(link4_dh.get_trans())
        T_4_0 = T_3_0 * T_4_3

        j5 = get_angle(T_PinchJoint_0.p - T_PalmJoint_0.p,
                       T_4_0.M.UnitZ(), up_vector=-T_4_0.M.UnitY())

        # Calculate j6
        # This too should be simple, compute the angle between the Rz_7_0 and Rx_5_0.
        link5_dh = self.get_link_params(4)
        link5_dh.theta = j5
        T_5_4 = convert_mat_to_frame(link5_dh.get_trans())
        T_5_0 = T_4_0 * T_5_4

        j6 = get_angle(T_7_0.M.UnitZ(), T_5_0.M.UnitX(), up_vector=-T_5_0.M.UnitY())

        return [j1, j2, j3, j4, j5, j6]


def create_class_json(root_dir=dynamic_path, psm_type='classic', tool_id=None):
    assert type(tool_id) == int, "tool id must be an integer, please check the input"
    kinematics_json = PSMKinematicSolver(tool_id=tool_id)
    file_name = f'class_psm_{psm_type}_{str(tool_id)}.json'
    file_path = os.path.join(root_dir, file_name)
    with open(file_path, 'w') as f:
        json_w = jsonpickle.encode(kinematics_json, indent=4)
        f.write(json_w)
    print(f'Created {file_name}')


if __name__ == "__main__":
    create_class_json(tool_id=420006)
    create_class_json(tool_id=400006)
