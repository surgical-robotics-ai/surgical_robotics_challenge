#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020-2021

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

import rospy
from ambf_client import Client
import psm_arm
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform
from PyKDL import Rotation, Vector, Frame


def rot_mat_to_quat(cp):
    R = Rotation(cp[0, 0], cp[0, 1], cp[0, 2],
                 cp[1, 0], cp[1, 1], cp[1, 2],
                 cp[2, 0], cp[2, 1], cp[2, 2])

    return R.GetQuaternion()


def np_mat_to_transform(cp):
    trans = Transform()
    trans.translation.x = cp[0, 3]
    trans.translation.y = cp[1, 3]
    trans.translation.z = cp[2, 3]

    Quat = rot_mat_to_quat(cp)

    trans.rotation.x = Quat[0]
    trans.rotation.y = Quat[1]
    trans.rotation.z = Quat[2]
    trans.rotation.w = Quat[3]
    return trans


def transform_to_frame(cp):
    frame = Frame()
    frame.p = Vector(cp.translation.x,
                     cp.translation.y,
                     cp.translation.z)
    frame.M = Rotation.Quaternion(cp.rotation.x,
                                  cp.rotation.y,
                                  cp.rotation.z,
                                  cp.rotation.w)
    return frame


class Options:
    run_psm1 = False
    run_psm2 = False
    run_psm3 = False
    namespace = '/CRTK'
    rate = 60


class ArmCRTKWrapper:
    def __init__(self, client, name, namespace):
        self.arm_name = name
        self.namespace = namespace
        self.arm = psm_arm.PSM(client, name)
        time.sleep(0.1)

        self.measured_jp_pub = rospy.Publisher(namespace + '/' + name + '/' + 'measured_js', JointState,
                                      queue_size=1)
        self.measured_cp_pub = rospy.Publisher(namespace + '/' + name + '/' + 'measured_cp', TransformStamped,
                                      queue_size=1)
        self.servo_jp_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'servo_jp', JointState,
                                             self.servo_jp_cb, queue_size=1)
        self.servo_jaw_jp_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'servo_jaw_jp', JointState,
                                             self.servo_jaw_jp_cb, queue_size=1)
        self.servo_cp_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'servo_cp', TransformStamped,
                                             self.servo_cp_cb, queue_size=1)
        self._measured_jp_msg = JointState()
        self._measured_jp_msg.name = self.arm.get_joint_names()

        self._measured_cp_msg = TransformStamped()
        self._measured_cp_msg.header.frame_id = 'baselink'
        self._jaw_angle = 0.5

    def servo_cp_cb(self, cp):
        frame = transform_to_frame(cp.transform)
        self.arm.servo_cp(frame)

    def servo_jp_cb(self, jp):
        self.arm.servo_jp(jp.position)

    def servo_jaw_jp_cb(self, jp):
        self._jaw_angle = jp.position[0]

    def run(self):
        self._measured_jp_msg.position = self.arm.measured_jp()
        self.measured_jp_pub.publish(self._measured_jp_msg)

        self._measured_cp_msg.transform = np_mat_to_transform(self.arm.measured_cp())
        self.measured_cp_pub.publish(self._measured_cp_msg)

        self.arm.set_jaw_angle(self._jaw_angle)
        self.arm.run_grasp_logic(self._jaw_angle)


class CRTK:
    def __init__(self, options):
        if options.run_psm1 is False and options.run_psm2 is False and options.run_psm3 is False:
            raise "Atleast one PSM arm has to be specified"

        self.client = Client("ambf_surgical_sim_crtk_node")
        self.client.connect()
        time.sleep(0.2)
        self._psms = []
        if options.run_psm1 is True:
            print("Launching CRTK-ROS Interface for PSM1 ")
            psm1 = ArmCRTKWrapper(self.client, 'psm1', options.namespace)
            self._psms.append(psm1)
        if options.run_psm2 is True:
            print("Launching CRTK-ROS Interface for PSM2 ")
            psm2 = ArmCRTKWrapper(self.client, 'psm2', options.namespace)
            self._psms.append(psm2)
        if options.run_psm3 is True:
            print("Launching CRTK-ROS Interface for PSM3 ")
            psm3 = ArmCRTKWrapper(self.client, 'psm3', options.namespace)
            self._psms.append(psm3)

        self._rate = rospy.Rate(options.rate)

    def run(self):
        while not rospy.is_shutdown():
            for arm in self._psms:
                arm.run()
            self._rate.sleep()


options = Options()
options.run_psm1 = True
options.run_psm2 = False
options.run_psm3 = False

crtk = CRTK(options)
crtk.run()




