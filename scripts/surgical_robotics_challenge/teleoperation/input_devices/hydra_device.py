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
#     \author    Haoying Zhou
#     \version   1.0
# */
# //==============================================================================

import PyKDL
from PyKDL import Frame, Rotation, Vector
from razer_hydra.msg import Hydra
from geometry_msgs.msg import PoseStamped, Twist
import rospy
import time
from scipy.spatial.transform import Rotation as Rot
import numpy as np

# Utilities


def kdl_frame_to_pose_msg(kdl_pose):
    #### rotation 0-3 w,x,y,z
    #### translation 0-2 x,y,z
    ps = PoseStamped()
    p = ps.pose
    p.position.x = kdl_pose.p[0]
    p.position.y = kdl_pose.p[1]
    p.position.z = kdl_pose.p[2]

    p.orientation.x = kdl_pose.M.GetQuaternion()[0]
    p.orientation.y = kdl_pose.M.GetQuaternion()[1]
    p.orientation.z = kdl_pose.M.GetQuaternion()[2]
    p.orientation.w = kdl_pose.M.GetQuaternion()[3]

    return ps


def pose_msg_to_kdl_frame(msg_pose):
    pose = msg_pose.pose
    f = Frame()
    f.p[0] = pose.position.x
    f.p[1] = pose.position.y
    f.p[2] = pose.position.z
    f.M = Rotation.Quaternion(pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z,
                              pose.orientation.w)

    return f


def hydra_msg_to_kdl_frame(msg_hydra):
    pose = msg_hydra.paddles[0].transform
    f = Frame()
    f.p[0] = pose.translation.x
    f.p[1] = pose.translation.y
    f.p[2] = pose.translation.z
    f.M = Rotation.Quaternion(pose.rotation.x,
                              pose.rotation.y,
                              pose.rotation.z,
                              pose.rotation.w)
    return f


# Init everything related to Geomagic
class HydraDevice:
    # The name should include the full qualified prefix. I.e. '/Geomagic/', or '/omniR_' etc.
    def __init__(self, name='hydra_calib', hydra_idx=0):
        ### hydra_idx=0 --> left, hydra_idx=1 --> right
        self.pose_topic_name = name
        assert (hydra_idx == 0) or (hydra_idx == 1), 'Incorrect hydra controller index'
        self.hydra_idx = hydra_idx
        self._active = False
        self._scale = [1.0, 1.0, 1.0]
        self._jaw_scale = 1.0
        self.clutch_button_pressed = False  # Used as Position Engage Clutch
        self.gripper_button_pressed = False  # Used as Gripper Open Close Binary Angle
        self.reset_pos = [0.0, 0.0, 0.0]
        self.reset_mtx = np.eye(3)
        ### if you would like to init the device before picking up, you may uncomment following lines. 
        # init_msg = rospy.wait_for_message(self.pose_topic_name, Hydra, timeout=1)
        # self.set_reset_pos(init_msg)
        self.is_reset_rot = False
        self._pose_sub = rospy.Subscriber(self.pose_topic_name, Hydra, self.pose_cb, queue_size=1)
        self._twist_sub = rospy.Subscriber(self.pose_topic_name, Hydra, self.twist_cb, queue_size=1)
        self.pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.jaw = 0.0
        self.reset_button = False
        self.pos_pre = [0.0, 0.0, 0.0]

        ### old script
        self.switch_psm = False
        self.pose_hydra = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.twist = PyKDL.Twist()
        # This offset is to align the pitch with the view frame
        R_off = Rotation.RPY(0.0, 0, 0)
        self._T_baseoffset = Frame(R_off, Vector(0, 0, 0))
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        self._T_tipoffset = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self._button_msg_time = rospy.Time.now()
        self._switch_psm_duration = rospy.Duration(0.5)

        if hydra_idx == 0:
            hydra_name = 'Left Paddle'
        else:
            hydra_name = 'Right Paddle'

        print('Creating Razer Device Named: ', name, 'of ', hydra_name, ' From ROS Topics')
        self._msg_counter = 0

    def set_base_frame(self, frame):
        self._T_baseoffset = frame
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        pass

    def set_tip_frame(self, frame):
        self._T_tipoffset = frame
        pass

    def set_scale(self, scale):
        self._scale = scale

    def get_scale(self):
        return self._scale

    def set_reset_rot_opt(self, msg):
        data = msg.paddles[self.hydra_idx]
        if data.buttons[2]:
            self.is_reset_rot = True
        else:
            self.is_reset_rot = False
        pass

    def set_reset_frame(self, msg):
        data = msg.paddles[self.hydra_idx]
        if data.buttons[0]:
            mtx_read = Rot.from_quat([data.transform.rotation.x,
                                      data.transform.rotation.y,
                                      data.transform.rotation.z,
                                      data.transform.rotation.w])
            self.reset_mtx = mtx_read.as_matrix()
            self.reset_pos = [data.transform.translation.x,
                              data.transform.translation.y,
                              data.transform.translation.z]
            self.reset_button = True
        else:
            self.reset_button = False
        pass

    def set_reset_pos(self, msg):
        data = msg.paddles[self.hydra_idx]
        self.reset_pos = [data.transform.translation.x,
                          data.transform.translation.y,
                          data.transform.translation.z]
        pass

    def get_clutch(self, msg):
        data = msg.paddles[self.hydra_idx]
        if data.buttons[5]:
            self.clutch_button_pressed = True
        else:
            self.clutch_button_pressed = False
        pass

    def hydra_msg_read(self, msg_hydra):
        self.set_reset_frame(msg_hydra)
        data = msg_hydra.paddles[self.hydra_idx].transform
        pos_temp = [data.translation.x,
                    data.translation.y,
                    data.translation.z]
        rot_temp = Rot.from_quat([data.rotation.x,
                                  data.rotation.y,
                                  data.rotation.z,
                                  data.rotation.w])
        return pos_temp, rot_temp.as_matrix()

    def pose_cb(self, msg):
        ## new script
        self._active = True
        self.get_clutch(msg)
        pos_temp, rot_temp = self.hydra_msg_read(msg)
        # print(msg.paddles[0].transform.translation.x)
        pose_output = []
        pos_init = [x-y for x, y in zip(pos_temp, self.reset_pos)]
        pos_output = [x*y for x, y in zip(self._scale, pos_init)]
        pose_output.extend(pos_output)
        mtx_temp = Rot.from_matrix(np.dot(np.transpose(self.reset_mtx), rot_temp))
        ori_output = mtx_temp.as_euler('xyz', degrees=False)
        pose_output.extend(ori_output)
        self.pose = pose_output
        self.jaw = self._jaw_scale * msg.paddles[self.hydra_idx].trigger
        self.hydra_pose_to_kdl_frame()
        pass

    def twist_cb(self, msg):
        twist = PyKDL.Twist()
        self.set_reset_frame(msg)
        data = msg.paddles[self.hydra_idx].transform
        pos_temp = [-(data.translation.y - self.reset_pos[1]) * 0.226,
                    (data.translation.z - self.reset_pos[2]) * 0.25,
                    -(data.translation.x - self.reset_pos[0]) * 0.19]
        vel = [x-y for x, y in zip(pos_temp, self.pos_pre)]
        self.pos_pre = pos_temp
        vel_out = [x if x <= 0.005 else 0.005 for x in vel]
        twist[0] = vel_out[0]
        twist[1] = vel_out[1]
        twist[2] = vel_out[2]
        twist[3] = 0
        twist[4] = 0
        twist[5] = 0
        self.twist = self._T_baseoffset_inverse * twist
        pass

    # def buttons_cb(self, msg):
    #     self.gripper_button_pressed = msg.white_button
    #     self.clutch_button_pressed = msg.grey_button
    #
    #     if self.clutch_button_pressed:
    #         time_diff = rospy.Time.now() - self._button_msg_time
    #         if time_diff.to_sec() < self._switch_psm_duration.to_sec():
    #             print('Allow PSM Switch')
    #             self.switch_psm = True
    #         self._button_msg_time = rospy.Time.now()

    def hydra_pose_to_kdl_frame(self):
        cur_frame = PyKDL.Frame()
        cur_frame.p = PyKDL.Vector(-float(self.pose[1]) * 0.226,
                                   float(self.pose[2]) * 0.25,
                                   -float(self.pose[0]) * 0.19)
        cur_frame.M = PyKDL.Rotation.EulerZYX(-float(self.pose[3]),
                                              float(self.pose[5]),
                                              -float(self.pose[4]))
        self.pose_hydra = self._T_baseoffset_inverse * cur_frame * self._T_tipoffset

    def command_force(self, force):
        pass

    def measured_cp(self):
        return self.pose_hydra

    def measured_cv(self):
        return self.twist

    def get_jaw_angle(self):
        return 0.3 - self.jaw * 0.3
        # if self.gripper_button_pressed:
        #     return 0.0
        # else:
        #     return 0.3


def test():
    rospy.init_node('test_hydra')

    d = HydraDevice()

    while not rospy.is_shutdown():
        [r, p, y] = d.measured_cp().M.GetRPY()
        [p_x, p_y, p_z] = d.measured_cp().p
        print('x: ', p_x, ', Y: ', p_y, ', Z: ', p_z)

        f = 180.0 / 3.1404
        r = round(r * f, 2)
        p = round(p * f, 2)
        y = round(y * f, 2)
        print('Roll: ', r, ', Pitch: ', p, ', Yaw: ', y)
        # tst = d.measured_cv()
        # print(tst.vel)
        time.sleep(1)


def test_np():
    rospy.init_node('test_hydra')

    d = HydraDevice()

    while not rospy.is_shutdown():
        pose = d.measured_cp()
        print(pose)
        time.sleep(0.5)


if __name__ == '__main__':
    test()
    test_np()
