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
import numpy as np
import rospy
from sensor_msgs.msg import ChannelFloat32
from random import random

class JointErrorsModel:
    def __init__(self, arm_name, num_joints):
        self._arm_name = arm_name
        self.num_jnts = num_joints
        self._joint_errors = [0.] * num_joints
        # Subscriber to set errors on the fly
        self._errors_sub = rospy.Subscriber('/ambf/env/' + arm_name + '/errors_model/set_errors',
                                            ChannelFloat32, self._errors_sub, queue_size=1)

    def _errors_sub(self, msg):
        errors = msg.values
        if type(errors) == tuple:
            errors = list(errors)
        self.set_errors(errors)

    def generate_random_from_max_value(self, max_errors_list):
        """
        # Set each joint error to a random value with a max range provided
        # by max_errors_list
        :param max_errors_list:
        """
        for i in range(self.num_jnts):
            rand_val = 2. * random() - 1.
            self._joint_errors[i] = rand_val * max_errors_list[i]
        print('Joint Errors: ', self._joint_errors)

    def set_errors(self, errors_list):
        """
        # Directly set the joint errors.
        :param errors_list:
        """
        for i in range(len(errors_list)):
            self._joint_errors[i] = errors_list[i]
        print('Joint Errors: ', self._joint_errors)

    def _size_check(self, q, joint_mask):
        qs_size = len(q)
        jnt_mask_size = len(joint_mask)
        if qs_size > self.num_jnts:
            print("ERROR! size of joint positions: ", qs_size, " > num of joints: ", self.num_jnts)
            print("IGNORING! ")
            return False

        if jnt_mask_size > qs_size:
            print("ERROR! JOINT MASK: ", joint_mask, " > size of joint positions: ", qs_size)
            print("IGNORING! ")
            return False

        return True

    def add_to_joints(self, q, joint_mask):
        if self._size_check(q, joint_mask):
            if type(q) == tuple:
                q = list(q)
            for i in range(len(joint_mask)):
                if joint_mask[i]:
                    q[i] = q[i] + self._joint_errors[i]
        return q

    def remove_from_joints(self, q, joint_mask):
        if self._size_check(q, joint_mask):
            if type(q) == tuple:
                q = list(q)
            for i in range(len(joint_mask)):
                if joint_mask[i]:
                    q[i] = q[i] - self._joint_errors[i]
        return q