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

from PyKDL import Frame, Rotation, Vector, Twist
import numpy as np
from surgical_robotics_challenge.utils.utilities import cartesian_interpolate_step
from surgical_robotics_challenge.utils.task3_init import NeedleInitialization
import time


class Scene:
    def __init__(self, simulation_manager):
        self.simulation_manager = simulation_manager
        self._needle = self.simulation_manager.get_obj_handle("Needle")
        self._entry1 = self.simulation_manager.get_obj_handle("Entry1")
        self._entry2 = self.simulation_manager.get_obj_handle("Entry2")
        self._entry3 = self.simulation_manager.get_obj_handle("Entry3")
        self._entry4 = self.simulation_manager.get_obj_handle("Entry4")
        self._exit1 = self.simulation_manager.get_obj_handle("Exit1")
        self._exit2 = self.simulation_manager.get_obj_handle("Exit2")
        self._exit3 = self.simulation_manager.get_obj_handle("Exit3")
        self._exit4 = self.simulation_manager.get_obj_handle("Exit4")
        time.sleep(0.1)

    def needle_measured_cp(self):
        return self._needle.get_pose()

    def entry1_measured_cp(self):
        return self._entry1.get_pose()

    def entry2_measured_cp(self):
        return self._entry2.get_pose()

    def entry3_measured_cp(self):
        return self._entry3.get_pose()

    def entry4_measured_cp(self):
        return self._entry4.get_pose()

    def exit1_measured_cp(self):
        return self._exit1.get_pose()

    def exit2_measured_cp(self):
        return self._exit2.get_pose()

    def exit3_measured_cp(self):
        return self._exit3.get_pose()

    def exit4_measured_cp(self):
        return self._exit4.get_pose()

    def task_3_setup_init(self, psm):
        ni = NeedleInitialization(self.simulation_manager)
        psm_tip = self.simulation_manager.get_obj_handle(psm.name + '/toolyawlink')
        # First we shall move the PSM to its initial pose using joint commands OR pose command
        psm.servo_jp([-0.4, -0.22, 0.139, -1.64, -0.37, -0.11])
        # Open the Jaws
        psm.set_jaw_angle(0.8)
        # Sleep to achieve the target pose and jaw angle
        time.sleep(0.5)
        ni.move_to(psm_tip)
        time.sleep(1.0)
        for i in range(30):
            # Close the jaws to grasp the needle
            # Calling it repeatedly a few times so that the needle is forced
            # between the gripper tips and grasped properly
            psm.set_jaw_angle(0.0)
            time.sleep(0.01)
        time.sleep(0.5)
        ni.release()
