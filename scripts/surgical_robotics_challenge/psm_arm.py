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
#     \author    Annie Huang
#     \version   1.0
# */
# //==============================================================================
from surgical_robotics_challenge.kinematics.psmKinematics import *
from surgical_robotics_challenge.utils.joint_errors_model import JointErrorsModel
from surgical_robotics_challenge.utils import coordinate_frames
import rospy
import time
from threading import Thread, Lock
from surgical_robotics_challenge.utils.interpolation import Interpolation


class PSMJointMapping:
    def __init__(self):
        self.idx_to_name = {0: 'baselink-yawlink',
                            1: 'yawlink-pitchbacklink',
                            2: 'pitchendlink-maininsertionlink',
                            3: 'maininsertionlink-toolrolllink',
                            4: 'toolrolllink-toolpitchlink',
                            5: 'toolpitchlink-toolyawlink'}

        self.name_to_idx = {'baselink-yawlink': 0,
                            'yawlink-pitchbacklink': 1,
                            'pitchendlink-maininsertionlink': 2,
                            'maininsertionlink-toolrolllink': 3,
                            'toolrolllink-toolpitchlink': 4,
                            'toolpitchlink-toolyawlink': 5}


pjm = PSMJointMapping()


class PSM:
    def __init__(self, simulation_manager, name, add_joint_errors=False, tool_id=PSMType.Default):
        self.simulation_manager = simulation_manager
        self.name = name
        assert tool_id is not None, 'Please specify a tool id'
        self.tool_id = int(tool_id)
        self.base = self.simulation_manager.get_obj_handle(name + '/baselink')
        self.base.set_joint_types([JointType.REVOLUTE, JointType.REVOLUTE, JointType.PRISMATIC, JointType.REVOLUTE,
                                   JointType.REVOLUTE, JointType.REVOLUTE, JointType.REVOLUTE, JointType.REVOLUTE])
        self.target_IK = self.simulation_manager.get_obj_handle(name + '_target_ik')
        self.palm_joint_IK = self.simulation_manager.get_obj_handle(name + '_palm_joint_ik')
        self.target_FK = self.simulation_manager.get_obj_handle(name + '_target_fk')
        self.sensor = self.simulation_manager._client.get_obj_handle(name + '/Sensor0')
        self.actuators = []
        self.actuators.append(self.simulation_manager._client.get_obj_handle(name + '/Actuator0'))
        time.sleep(0.5)
        self.grasped = [False, False, False]
        self.graspable_objs_prefix = ["Needle", "Thread", "Puzzle"]
        self.T_t_b_home = coordinate_frames.PSM.T_t_b_home
        self._kd = PSMKinematicSolver(psm_type=self.tool_id, tool_id=self.tool_id)

        # Transform of Base in World
        self._T_b_w = None
        # Transform of World in Base
        self._T_w_b = None
        self._base_pose_updated = False
        self._num_joints = 6
        self._ik_solution = np.zeros([self._num_joints])
        self._last_jp = np.zeros([self._num_joints])
        self._joints_error_mask = [1, 1, 1, 0, 0, 0]  # Only apply error to joints with 1's
        self._joint_error_model = JointErrorsModel(self.name, self._num_joints)
        self.interpolater = Interpolation()
        self._force_exit_thread = False
        self._thread_lock = Lock()
        if add_joint_errors:
            max_errors_list = [0.] * self._num_joints  # No error
            max_errors_list[0] = np.deg2rad(5.0)  # Max Error Joint 0 -> +-5 deg
            max_errors_list[1] = np.deg2rad(5.0)  # Max Error Joint 1 -> +- 5 deg
            max_errors_list[2] = 0.005  # Max Error Joint 2 -> +- 5 mm or 0.05 Simulation units
            self._joint_error_model.generate_random_from_max_value(max_errors_list)

        # Initialize Jaw Angle
        self.set_jaw_angle(0.5)

    def set_home_pose(self, pose):
        self.T_t_b_home = pose

    def is_present(self):
        if self.base is None:
            return False
        else:
            return True

    def get_ik_solution(self):
        return self._ik_solution

    def get_lower_limits(self):
        return self._kd.lower_limits

    def get_upper_limits(self):
        return self._kd.upper_limits

    def get_T_b_w(self):
        self._update_base_pose()
        return self._T_b_w

    def get_T_w_b(self):
        self._update_base_pose()
        return self._T_w_b

    def _update_base_pose(self):
        if not self._base_pose_updated:
            self._T_b_w = self.base.get_pose()
            self._T_w_b = self._T_b_w.Inverse()
            self._base_pose_updated = True

    def run_grasp_logic(self, jaw_angle):
        for i in range(len(self.actuators)):
            if jaw_angle <= 0.2:
                if self.sensor is not None:
                    if self.sensor.is_triggered(i):
                        sensed_obj = self.sensor.get_sensed_object(i)
                        for s in self.graspable_objs_prefix:
                            if s in sensed_obj:
                                if not self.grasped[i]:
                                    qualified_name = sensed_obj
                                    self.actuators[i].actuate(qualified_name)
                                    self.grasped[i] = True
                                    print('Grasping Sensed Object Names', sensed_obj)
            else:
                if self.actuators[i] is not None:
                    self.actuators[i].deactuate()
                    if self.grasped[i] is True:
                        print('Releasing Grasped Object')
                    self.grasped[i] = False
                    # print('Releasing Actuator ', i)

    def servo_cp(self, T_t_b):
        if type(T_t_b) in [np.matrix, np.array]:
            T_t_b = convert_mat_to_frame(T_t_b)

        ik_solution = self._kd.compute_IK(T_t_b)
        self._ik_solution = enforce_limits(ik_solution, self.get_lower_limits(), self.get_upper_limits())
        self.servo_jp(self._ik_solution)

    def move_cp(self, T_t_b, execute_time=0.5, control_rate=120):
        if type(T_t_b) in [np.matrix, np.array]:
            T_t_b = convert_mat_to_frame(T_t_b)

        ik_solution = self._kd.compute_IK(T_t_b)
        self._ik_solution = enforce_limits(ik_solution, self.get_lower_limits(), self.get_upper_limits())
        self.move_jp(self._ik_solution, execute_time, control_rate)

    def servo_cv(self, twist):
        pass

    def optimize_jp(self, jp):
        # Optimizing the tool shaft roll angle
        pass

    def servo_jp(self, jp):
        jp = self._joint_error_model.add_to_joints(jp, self._joints_error_mask)
        self.base.set_joint_pos(0, jp[0])
        self.base.set_joint_pos(1, jp[1])
        self.base.set_joint_pos(2, jp[2])
        self.base.set_joint_pos(3, jp[3])
        self.base.set_joint_pos(4, jp[4])
        self.base.set_joint_pos(5, jp[5])

    def move_jp(self, jp_cmd, execute_time=0.5, control_rate=120):

        jp_cur = self.measured_jp()
        jv_cur = self.measured_jv()

        zero = np.zeros(6)
        self.interpolater.compute_interpolation_params(jp_cur, jp_cmd, jv_cur, zero, zero, zero, 0, execute_time)
        trajectory_execute_thread = Thread(target=self._execute_trajectory, args=(self.interpolater, execute_time, control_rate,))
        self._force_exit_thread = True
        trajectory_execute_thread.start()
    
    def _execute_trajectory(self, trajectory_gen, execute_time, control_rate):
        self._thread_lock.acquire()
        self._force_exit_thread = False
        init_time = rospy.Time.now().to_sec()
        control_rate = rospy.Rate(control_rate)
        while not rospy.is_shutdown() and not self._force_exit_thread:
            cur_time = rospy.Time.now().to_sec() - init_time
            if cur_time > execute_time:
                break
            val = trajectory_gen.get_interpolated_x(np.array(cur_time, dtype=np.float32))
            self.servo_jp(val)
            control_rate.sleep()
        self._thread_lock.release()

    def servo_jv(self, jv):
        print("Setting Joint Vel", jv)
        self.base.set_joint_vel(0, jv[0])
        self.base.set_joint_vel(1, jv[1])
        self.base.set_joint_vel(2, jv[2])
        self.base.set_joint_vel(3, jv[3])
        self.base.set_joint_vel(4, jv[4])
        self.base.set_joint_vel(5, jv[5])

    def set_jaw_angle(self, jaw_angle):
        self.base.set_joint_pos(6, jaw_angle)
        self.base.set_joint_pos(7, jaw_angle)
        self.run_grasp_logic(jaw_angle)

    def measured_cp(self):
        jp = self.measured_jp()
        jp.append(0.0)
        return self._kd.compute_FK(jp, 7)

    def measured_jp(self):
        j0 = self.base.get_joint_pos(0)
        j1 = self.base.get_joint_pos(1)
        j2 = self.base.get_joint_pos(2)
        j3 = self.base.get_joint_pos(3)
        j4 = self.base.get_joint_pos(4)
        j5 = self.base.get_joint_pos(5)
        q = [j0, j1, j2, j3, j4, j5]
        q = self._joint_error_model.remove_from_joints(q, self._joints_error_mask)
        return q

    def measured_jv(self):
        j0 = self.base.get_joint_vel(0)
        j1 = self.base.get_joint_vel(1)
        j2 = self.base.get_joint_vel(2)
        j3 = self.base.get_joint_vel(3)
        j4 = self.base.get_joint_vel(4)
        j5 = self.base.get_joint_vel(5)
        return [j0, j1, j2, j3, j4, j5]

    def get_joint_names(self):
        return self.base.get_joint_names()
