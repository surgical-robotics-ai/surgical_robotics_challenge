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
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
import obj_control_gui


def test_ik():
    # We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
    # in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)
    test_q = [-0.3, 0.2, 0.1, -0.9, 0.0, 0.0, 0.0]
    T_7_0 = compute_FK(test_q)
    computed_q = compute_IK(convert_mat_to_frame(T_7_0))
    for i in range(0, 6):
        computed_q[i] = round(computed_q[i], 4)

    print('Test Q: ', test_q[0:6])
    print('Comp Q: ', computed_q)


class PSM:
    def __init__(self, client, namespace, OG, scale, init_xyz=None):
        self.c = client
        self.scale = scale
        self.base = self.c.get_obj_handle(namespace + '/baselink') 
        self.target_IK = self.c.get_obj_handle(namespace + '_target_ik')
        self.palm_joint_IK = self.c.get_obj_handle(namespace + '_palm_joint_ik')
        self.target_FK = self.c.get_obj_handle(namespace + '_target_fk')
        self.sensor = self.c.get_obj_handle(namespace + '/Sensor0')
        self.actuators = []
        self.actuators.append(self.c.get_obj_handle(namespace + '/Actuator0'))
        self.actuators.append(self.c.get_obj_handle(namespace + '/Actuator1'))
        self.actuators.append(self.c.get_obj_handle(namespace + '/Actuator2'))
        self.grasped = []
        self.grasped.append(False)
        self.grasped.append(False)
        self.grasped.append(False)
        self.obj_gui = OG
        if init_xyz is None:
            base_pos = self.base.get_pos()
            print "Base Pos 2"
            print base_pos
            init_xyz = [base_pos.x, base_pos.y, base_pos.z - 0.10 * self.scale]
        self.obj_gui.set_init_xyz(init_xyz)

    
class TestPSMIK:
    def __init__(self, run_psm_one, run_psm_two, run_psm_three, OG1=None, OG2=None, OG3=None):
        self.c = Client()
        self.c.connect()

        self.psm1_scale = 10.0
        self.psm2_scale = 10.0
        self.psm3_scale = 10.0
        self.counter = 0

        self.run_psm_one = run_psm_one
        self.run_psm_two = run_psm_two
        self.run_psm_three = run_psm_three

        time.sleep(1.0)

        if self.run_psm_one is True:
            print('PREPARING TO LOAD IK FOR PSM1')
            init_xyz = [0.1, -0.85, -0.15]
            # init_xyz = [0.65256, 1.36340, -0.5]
            self.PSM1 = PSM(self.c, 'psm1', OG1, self.psm1_scale, init_xyz)
           
        if self.run_psm_two is True:
            init_xyz = [-0.1, -0.8, -0.15]
            print('PREPARING TO LOAD IK FOR PSM2')
            self.PSM2 = PSM(self.c, 'psm2', OG2, self.psm2_scale, init_xyz)

        if self.run_psm_three is True:
            print('PREPARING TO LOAD IK FOR PSM3')
            init_xyz = [0, -1.0, -0.1]
            self.PSM3 = PSM(self.c, 'psm3', OG3, self.psm3_scale, init_xyz)

        if not run_psm_one and not run_psm_two and not run_psm_three:
            print('YOU HAVE TO RUN ATLEAST ONE PSMS IK FOR THIS SCRIPT TO DO ANYTHING')

        # The following are the names of the controllable joints.
        #  'baselink-yawlink', 0
        #  'yawlink-pitchbacklink', 1
        #  'pitchendlink-maininsertionlink', 2
        #  'maininsertionlink-toolrolllink', 3
        #  'toolrolllink-toolpitchlink', 4
        #  'toolpitchlink-toolyawlink', 5
        #  'toolyawlink-toolgripper1link', 6a
        #  'toolyawlink-toolgripper2link', 6b

    def runOneArm(self, arm):
        arm.obj_gui.App.update()
        # Move the Target Position Based on the GUI
        x = arm.obj_gui.x
        y = arm.obj_gui.y
        z = arm.obj_gui.z
        ro = arm.obj_gui.ro
        pi = arm.obj_gui.pi
        ya = arm.obj_gui.ya
        gr = arm.obj_gui.gr
        if arm.target_IK is not None:
            arm.target_IK.set_pos(x, y, z)
            arm.target_IK.set_rpy(ro, pi, ya)
            # euler = Rotation.RPY(ro, pi, ya)
            # arm.target_IK.set_rpy(euler.GetRPY()[0], euler.GetRPY()[1], euler.GetRPY()[2])

        P_t_w = Vector(x, y, z)
        R_t_w = Rotation.RPY(ro, pi, ya)
        # R_t_w = Rotation.EulerZYX(ya, pi, ro)
        T_t_w = Frame(R_t_w, P_t_w)

        p = arm.base.get_pos()
        q = arm.base.get_rot()
        P_b_w = Vector(p.x, p.y, p.z)
        R_b_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
        T_b_w = Frame(R_b_w, P_b_w)
        T_t_b = T_b_w.Inverse() * T_t_w
        # P_t_b = T_t_b.p
        # P_t_b_scaled = P_t_b / self.psm1_scale
        # T_t_b.p = P_t_b_scaled
        computed_q = enforce_limits(compute_IK(T_t_b))

        if arm.palm_joint_IK is not None:
            T_PalmJoint_0 = get_T_PalmJoint_0()
            if T_PalmJoint_0 is not None:
                T_PalmJoint_w = T_b_w * T_PalmJoint_0
                arm.palm_joint_IK.set_pos(T_PalmJoint_w.p[0], T_PalmJoint_w.p[1], T_PalmJoint_w.p[2])
                arm.palm_joint_IK.set_rpy(T_PalmJoint_w.M.GetRPY()[0], T_PalmJoint_w.M.GetRPY()[1], T_PalmJoint_w.M.GetRPY()[2])
            else:
                print "T_PalmJoint_0 is Not Set"

        if arm.target_FK is not None:
            computed_q.append(0)
            T_7_0 = convert_mat_to_frame(compute_FK(computed_q))
            T_7_w = T_b_w * T_7_0
            P_7_0 = T_7_w.p
            RPY_7_0 = T_7_w.M.GetRPY()
            arm.target_FK.set_pos(P_7_0[0], P_7_0[1], P_7_0[2])
            arm.target_FK.set_rpy(RPY_7_0[0], RPY_7_0[1], RPY_7_0[2])

        # self.counter = self.counter + 1
        # if self.counter % 10 == 0:
        #     self.counter = 0
        #     print ["{0:0.2f}".format(i) for i in computed_q]

        arm.base.set_joint_pos('baselink-yawlink', computed_q[0])
        arm.base.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
        arm.base.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
        arm.base.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
        arm.base.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
        arm.base.set_joint_pos('toolpitchlink-toolyawlink', computed_q[5])
        arm.base.set_joint_pos('toolyawlink-toolgripper1link', gr)
        arm.base.set_joint_pos('toolyawlink-toolgripper2link', gr)

        for i in range(3):
            if gr <= 0.2:
                if arm.sensor is not None:
                    if arm.sensor.is_triggered(i):
                        sensed_obj = arm.sensor.get_sensed_object(i)
                        if sensed_obj == 'Needle' or 'T' in sensed_obj:
                            if not arm.grasped[i]:
                                qualified_nane = '/ambf/env/BODY ' + sensed_obj
                                arm.actuators[i].actuate(qualified_nane)
                                arm.grasped[i] = True
                                print('Grasping Sensed Object Names', sensed_obj)
            else:
                if arm.actuators[i] is not None:
                    arm.actuators[i].deactuate()
                    if arm.grasped[i] is True:
                        print('Releasing Grasped Object')
                    arm.grasped[i] = False
                    # print('Releasing Actuator ', i)
            
    def run(self):
        while not rospy.is_shutdown():

            if self.run_psm_one is True:
                self.runOneArm(self.PSM1)
                
            if self.run_psm_two is True:
                self.runOneArm(self.PSM2)

            if self.run_psm_three is True:
                self.runOneArm(self.PSM3)
                
            time.sleep(0.005)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=True)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print parsed_args

    OG1 = None
    OG2 = None
    OG3 = None

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
    
    if parsed_args.run_psm_one is True:
        # Initial Target Offset for PSM1
        psm1_xyz = [0.0, 0.0, 0.0]
        psm1_rpy = [3.14, 0, -1.57079]
        OG1 = obj_control_gui.ObjectGUI('psm1/baselink', psm1_xyz, psm1_rpy, 3.0, 3.14, 0.000001)

    if parsed_args.run_psm_two is True:
        # Initial Target Offset for PSM2
        psm2_xyz = [0.0, 0.0, 0.0]
        psm2_rpy = [3.14, 0, -1.57079]
        OG2 = obj_control_gui.ObjectGUI('psm2/baselink', psm2_xyz, psm2_rpy, 3.0, 3.14, 0.000001)

    if parsed_args.run_psm_three is True:
        # Initial Target Offset for PSM2
        psm3_xyz = [0.0, 0.0, 0.0]
        psm3_rpy = [3.14, 0, -1.57079]
        OG3 = obj_control_gui.ObjectGUI('psm3/baselink', psm3_xyz, psm3_rpy, 3.0, 3.14, 0.000001)

    psmIK = TestPSMIK(parsed_args.run_psm_one, parsed_args.run_psm_two, parsed_args.run_psm_three, OG1, OG2, OG3)
    psmIK.run()
