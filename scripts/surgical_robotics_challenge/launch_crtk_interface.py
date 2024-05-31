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
import rospy
import psm_arm
import ecm_arm
import scene
import time
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from PyKDL import Rotation, Vector, Frame
from argparse import ArgumentParser
from surgical_robotics_challenge.utils.utilities import *
from simulation_manager import SimulationManager
from enum import Enum


class Options:
    run_psm_one = True
    run_psm_two = True
    run_psm_three = False
    run_ecm = True
    run_scene = True
    namespace = '/CRTK'
    rate = 120
    tool_id = 400006


class PSMCRTKWrapper:
    def __init__(self, client, name, namespace, tool_id):
        self.arm_name = name
        self.namespace = namespace
        self.arm = psm_arm.PSM(client, name, add_joint_errors=True, tool_id=tool_id)
        time.sleep(0.1)

        self.measured_js_pub = rospy.Publisher(namespace + '/' + name + '/' + 'measured_js', JointState,
                                               queue_size=1)

        self.measured_cp_pub = rospy.Publisher(namespace + '/' + name + '/' + 'measured_cp', PoseStamped,
                                               queue_size=1)

        self.T_b_w_pub = rospy.Publisher(namespace + '/' + name + '/' + 'T_b_w', PoseStamped,
                                         queue_size=1)

        self.measured_cv_pub = rospy.Publisher(namespace + '/' + name + '/' + 'measured_cv', TwistStamped,
                                               queue_size=1)

        self.servo_jp_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'servo_jp', JointState,
                                             self.servo_jp_cb, queue_size=1)

        self.servo_jp_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'move_jp', JointState,
                                             self.move_jp_cb, queue_size=1)

        self.servo_jv_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'servo_jv', JointState,
                                             self.servo_jv_cb, queue_size=1)

        self.servo_cp_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'servo_cp', PoseStamped,
                                             self.servo_cp_cb, queue_size=1)

        self.servo_cp_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'move_cp', PoseStamped,
                                             self.move_cp_cb, queue_size=1)

        self.servo_jaw_jp_sub = rospy.Subscriber(namespace + '/' + name + '/jaw/' + 'servo_jp', JointState,
                                                 self.servo_jaw_jp_cb, queue_size=1)

        self._measured_js_msg = JointState()
        self._measured_js_msg.name = self.arm.get_joint_names()

        self._measured_cp_msg = PoseStamped()
        self._measured_cp_msg.header.frame_id = name + '/baselink'
        self._T_b_w_msg = PoseStamped()
        self._T_b_w_msg.header.frame_id = 'world'
        self._jaw_angle = 0.5

    def servo_cp_cb(self, cp):
        frame = pose_to_frame(cp.pose)
        self.arm.servo_cp(frame)

    def move_cp_cb(self, cp):
        frame = pose_to_frame(cp.pose)
        self.arm.move_cp(frame)

    def servo_jp_cb(self, js):
        self.arm.servo_jp(js.position)

    def move_jp_cb(self, js):
        self.arm.move_jp(js.position)

    def servo_jv_cb(self, js):
        self.arm.servo_jv(js.velocity)

    def servo_jaw_jp_cb(self, jp):
        self._jaw_angle = jp.position[0]
        # Set jaw angle and run grasp logic
        self.arm.set_jaw_angle(self._jaw_angle)
        self.arm.run_grasp_logic(self._jaw_angle)

    def publish_js(self):
        self._measured_js_msg.header.stamp = rospy.Time.now()
        self._measured_js_msg.position = self.arm.measured_jp()
        self._measured_js_msg.velocity = self.arm.measured_jv()
        self.measured_js_pub.publish(self._measured_js_msg)

    def publish_cs(self):
        self._measured_cp_msg.header.stamp = rospy.Time.now()
        self._measured_cp_msg.pose = np_mat_to_pose(self.arm.measured_cp())
        self.measured_cp_pub.publish(self._measured_cp_msg)

    def publish_T_b_w(self):
        self._T_b_w_msg.header.stamp = rospy.Time.now()
        self._T_b_w_msg.pose = np_mat_to_pose(self.arm.get_T_b_w())
        self.T_b_w_pub.publish(self._T_b_w_msg)

    def run(self):
        self.publish_js()
        self.publish_cs()
        self.publish_T_b_w()


class ECMCRTKWrapper:
    def __init__(self, client, name, namespace):
        self.arm_name = name
        self.namespace = namespace
        self.arm = ecm_arm.ECM(client, 'CameraFrame')
        self._servo_jp_cmd = [0.0, 0.0, 0.0, 0.0]
        time.sleep(0.1)

        self.measured_js_pub = rospy.Publisher(namespace + '/' + name + '/' + 'measured_js', JointState,
                                               queue_size=1)

        self.measured_cp_pub = rospy.Publisher(namespace + '/' + name + '/' + 'measured_cp', PoseStamped,
                                               queue_size=1)

        self.servo_jp_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'servo_jp', JointState,
                                             self.servo_jp_cb, queue_size=1)

        # self.servo_jv_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'servo_jv', JointState,
        #                                      self.servo_jv_cb, queue_size=1)

        # self.servo_cp_sub = rospy.Subscriber(namespace + '/' + name + '/' + 'servo_cp', PoseStamped,
        #                                      self.servo_cp_cb, queue_size=1)

        self._measured_js_msg = JointState()
        self._measured_js_msg.name = ["j0", "j1", "j2", "j3"]

        self._measured_cp_msg = PoseStamped()
        self._measured_cp_msg.header.frame_id = name + '/baselink'

        self._measured_cv_msg = TwistStamped()
        self._measured_cv_msg.header.frame_id = 'world'

    # def servo_cp_cb(self, cp):
    #     frame = pose_to_frame(cp.pose)
    #     self.arm.servo_cp(frame)

    def servo_jp_cb(self, js):
        self._servo_jp_cmd = js.position
        self.arm.servo_jp(self._servo_jp_cmd)

    # def servo_jv_cb(self, js):
    #     self.arm.servo_jv(js.velocity)

    def publish_js(self):
        self._measured_js_msg.header.stamp = rospy.Time.now()
        self._measured_js_msg.position = self.arm.measured_jp()
        self.measured_js_pub.publish(self._measured_js_msg)

    def publish_cs(self):
        self._measured_cp_msg.header.stamp = rospy.Time.now()
        self._measured_cp_msg.pose = np_mat_to_pose(self.arm.measured_cp())
        self.measured_cp_pub.publish(self._measured_cp_msg)

    def run(self):
        self.publish_js()
        self.publish_cs()


class SceneObjectType(Enum):
    Needle=1
    Entry1=2
    Entry2=3
    Entry3=4
    Entry4=5
    Exit1=6
    Exit2=7
    Exit3=8
    Exit4=9


class SceneCRTKWrapper:
    def __init__(self, simulation_manager, namespace):
        self.namespace = namespace
        self.scene = scene.Scene(simulation_manager)
        self._scene_object_pubs = dict()
        self._scene_object_pubs[SceneObjectType.Needle] = [None, self.scene.needle_measured_cp, PoseStamped()]
        self._scene_object_pubs[SceneObjectType.Entry1] = [None, self.scene.entry1_measured_cp, PoseStamped()]
        self._scene_object_pubs[SceneObjectType.Entry2] = [None, self.scene.entry2_measured_cp, PoseStamped()]
        self._scene_object_pubs[SceneObjectType.Entry3] = [None, self.scene.entry3_measured_cp, PoseStamped()]
        self._scene_object_pubs[SceneObjectType.Entry4] = [None, self.scene.entry4_measured_cp, PoseStamped()]
        self._scene_object_pubs[SceneObjectType.Exit1] = [None, self.scene.exit1_measured_cp, PoseStamped()]
        self._scene_object_pubs[SceneObjectType.Exit2] = [None, self.scene.exit2_measured_cp, PoseStamped()]
        self._scene_object_pubs[SceneObjectType.Exit3] = [None, self.scene.exit3_measured_cp, PoseStamped()]
        self._scene_object_pubs[SceneObjectType.Exit4] = [None, self.scene.exit4_measured_cp, PoseStamped()]

        suffix = '/measured_cp'
        for k, i in self._scene_object_pubs.items():
            i[0] = rospy.Publisher(namespace + '/' + k.name + suffix, PoseStamped, queue_size=1)
            i[2].header.frame_id = 'world'

    def publish_cs(self):
        for k, i in self._scene_object_pubs.items():
            i[2].header.stamp = rospy.Time.now()
            i[2].pose = np_mat_to_pose(i[1]())
            i[0].publish(i[2])

    def run(self):
        self.publish_cs()


class SceneManager:
    def __init__(self, options):
        self.simulation_manager = SimulationManager("ambf_surgical_sim_crtk_node")
        time.sleep(0.2)
        self._components = []
        if options.run_psm_one is True:
            print("Launching CRTK-ROS Interface for PSM1 ")
            self.psm1 = PSMCRTKWrapper(self.simulation_manager, 'psm1', options.namespace, options.tool_id)
            self._components.append(self.psm1)
        if options.run_psm_two is True:
            print("Launching CRTK-ROS Interface for PSM2 ")
            self.psm2 = PSMCRTKWrapper(self.simulation_manager, 'psm2', options.namespace, options.tool_id)
            self._components.append(self.psm2)
        if options.run_psm_three is True:
            print("Launching CRTK-ROS Interface for PSM3 ")
            self.psm3 = PSMCRTKWrapper(self.simulation_manager, 'psm3', options.namespace, options.tool_id)
            self._components.append(self.psm3)
        if options.run_ecm:
            print("Launching CRTK-ROS Interface for ECM ")
            self.ecm = ECMCRTKWrapper(self.simulation_manager, 'ecm', options.namespace)
            self._components.append(self.ecm)
        if options.run_scene:
            print("Launching CRTK-ROS Interface for Scene ")
            self.scene = SceneCRTKWrapper(self.simulation_manager, options.namespace)
            self._components.append(self.scene)

        self._task_3_init_sub = rospy.Subscriber('/CRTK/scene/task_3_setup/init',
                                                Empty, self.task_3_setup_cb, queue_size=1)

        self._task_3_setup_reaady_pub = rospy.Publisher('/CRTK/scene/task_3_setup/ready', Empty, queue_size=1)

        self._rate = rospy.Rate(options.rate)

    def task_3_setup_cb(self, msg):
        print("CRTK-ROS Based: Task 3 Setup Called")
        self.scene.scene.task_3_setup_init(self.psm2.arm)
        self._task_3_setup_reaady_pub.publish(Empty())

    def run(self):
        while not rospy.is_shutdown():
            for comp in self._components:
                comp.run()
            self._rate.sleep()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--one', action='store', dest='run_psm_one', help='RUN PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='RUN PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='RUN PSM3', default=False)
    parser.add_argument('--ecm', action='store', dest='run_ecm', help='RUN ECM', default=True)
    parser.add_argument('--scene', action='store', dest='run_scene', help='RUN Scene', default=True)
    parser.add_argument('--ns', action='store', dest='namespace', help='Namespace', default='/CRTK')
    parser.add_argument('--rate', action='store', dest='rate', help='Rate of Publishing', default=120)
    parser.add_argument('--tool_id', action='store', dest='psm_tool_id', help='PSM Tool IDS', default=400006)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)
    options = Options()

    options.run_psm_one = get_boolean_from_opt(parsed_args.run_psm_one)
    options.run_psm_two = get_boolean_from_opt(parsed_args.run_psm_two)
    options.run_psm_three = get_boolean_from_opt(parsed_args.run_psm_three)
    options.run_ecm = get_boolean_from_opt(parsed_args.run_ecm)
    options.run_scene = get_boolean_from_opt(parsed_args.run_scene)

    options.namespace = parsed_args.namespace
    options.rate = parsed_args.rate
    options.tool_id = parsed_args.psm_tool_id

    sceneManager = SceneManager(options)
    sceneManager.run()
