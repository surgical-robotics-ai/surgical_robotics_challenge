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
from PyKDL import Frame, Rotation, Vector
import time


class PSM:
    def __init__(self, client, name):
        self.client = client
        self.name = name
        self.base = self.client.get_obj_handle(name + '/baselink')
        self.target_IK = self.client.get_obj_handle(name + '_target_ik')
        self.palm_joint_IK = self.client.get_obj_handle(name + '_palm_joint_ik')
        self.target_FK = self.client.get_obj_handle(name + '_target_fk')
        self.sensor = self.client.get_obj_handle(name + '/Sensor0')
        self.actuators = []
        self.actuators.append(self.client.get_obj_handle(name + '/Actuator0'))
        self.actuators.append(self.client.get_obj_handle(name + '/Actuator1'))
        self.actuators.append(self.client.get_obj_handle(name + '/Actuator2'))
        time.sleep(0.5)
        self.grasped = [False, False, False]

        self.T_t_b_desired = Frame(Rotation.RPY(3.14, 0.0, 1.57079), Vector(0, 0, -1.0))

        # Transform of Base in World
        self._T_b_w = None
        # Transform of World in Base
        self._T_w_b = None
        self._base_pose_updated = False
        self._num_joints = 6
        self._ik_solution = np.zeros([self._num_joints])
        self._last_jp = np.zeros([self._num_joints])

    def get_ik_solution(self):
        return self._ik_solution

    def get_T_b_w(self):
        self._update_base_pose()
        return self._T_b_w

    def get_T_w_b(self):
        self._update_base_pose()
        return self._T_w_b

    def _update_base_pose(self):
        if not self._base_pose_updated:
            p = self.base.get_pos()
            q = self.base.get_rot()
            P_b_w = Vector(p.x, p.y, p.z)
            R_b_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
            self._T_b_w = Frame(R_b_w, P_b_w)
            self._T_w_b = self._T_b_w.Inverse()
            self._base_pose_updated = True

    def run_grasp_logic(self, jaw_angle):
        for i in range(len(self.actuators)):
            if jaw_angle <= 0.2:
                if self.sensor is not None:
                    if self.sensor.is_triggered(i):
                        sensed_obj = self.sensor.get_sensed_object(i)
                        if sensed_obj == 'Needle' or 'T' in sensed_obj:
                            if not self.grasped[i]:
                                qualified_nane = '/ambf/env/BODY ' + sensed_obj
                                self.actuators[i].actuate(qualified_nane)
                                self.grasped[i] = True
                                print('Grasping Sensed Object Names', sensed_obj)
            else:
                if self.actuators[i] is not None:
                    self.actuators[i].deactuate()
                    if self.grasped[i] is True:
                        print('Releasing Grasped Object')
                    self.grasped[i] = False
                    # print('Releasing Actuator ', i)

    def move_cp(self, T_t_b):
        if type(T_t_b) in [np.matrix, np.array]:
            T_t_b = convert_mat_to_frame(T_t_b)

        ik_solution = compute_IK(T_t_b)
        self._ik_solution = enforce_limits(ik_solution)
        self.move_jp(self._ik_solution)

    def optimize_jp(self, jp):
        # Optimizing the tool shaft roll angle
        pass

    def move_jp(self, jp):
        self.base.set_joint_pos('baselink-yawlink', jp[0])
        self.base.set_joint_pos('yawlink-pitchbacklink', jp[1])
        self.base.set_joint_pos('pitchendlink-maininsertionlink', jp[2])
        self.base.set_joint_pos('maininsertionlink-toolrolllink', jp[3])
        self.base.set_joint_pos('toolrolllink-toolpitchlink', jp[4])
        self.base.set_joint_pos('toolpitchlink-toolyawlink', jp[5])

    def set_jaw_angle(self, jaw_angle):
        self.base.set_joint_pos('toolyawlink-toolgripper1link', jaw_angle)
        self.base.set_joint_pos('toolyawlink-toolgripper2link', jaw_angle)

    def measure_cp(self):
        pass

