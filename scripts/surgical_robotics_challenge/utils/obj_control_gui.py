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
import sys
if sys.version_info[0] >= 3:
    from tkinter import *
else:
    from Tkinter import *


class ObjectGUI:
    def __init__(self, obj_name, initial_xyz=[0, 0, 0], initial_rpy=[0, 0, 0], range_xyz=1.0, range_rpy=3.14, resolution=0.0001):
        self.App = Tk()
        self.initial_xyz = initial_xyz
        self.initial_rpy = initial_rpy
        self.range_xyz = range_xyz
        self.range_rpy = range_rpy
        self.resolution = resolution

        self.x = self.initial_xyz[0]
        self.y = self.initial_xyz[1]
        self.z = self.initial_xyz[2]
        self.ro = self.initial_rpy[0]
        self.pi = self.initial_rpy[1]
        self.ya = self.initial_rpy[2]
        self.gr = 0.0
        self.x_slider = None
        self.y_slider = None
        self.z_slider = None
        self.ro_slider = None
        self.pi_slider = None
        self.ya_slider = None
        self.gr_slider = None

        self.cartesian_mode = 0
        self.obj_name = obj_name
        self.create_gui(self.App, self.obj_name)

    def set_init_xyz(self, xyz):
        self.initial_xyz = xyz
        self.x = self.initial_xyz[0]
        self.y = self.initial_xyz[1]
        self.z = self.initial_xyz[2]
        print(self.initial_xyz)
        self.App.quit()
        self.App.destroy()
        self.App = Tk()
        self.create_gui(self.App, self.obj_name)

    def set_init_rpy(self, rpy):
        self.initial_rpy = rpy
        self.App.quit()
        self.App.destroy()
        del self.App
        self.App = Tk()
        self.create_gui(self.App, self.obj_name)

    # Def Init Function
    def get_app_handle(self):
        return self.App

    # Define Callbacks for Tkinter GUI Sliders
    def x_cb(self, val):
        self.x = float(val)

    def y_cb(self, val):
        self.y = float(val)

    def z_cb(self, val):
        self.z = float(val)

    def roll_cb(self, val):
        self.ro = float(val)

    def pitch_cb(self, val):
        self.pi = float(val)

    def yaw_cb(self, val):
        self.ya = float(val)

    def gr_cb(self, val):
        self.gr = float(val)

    def zero_all_cb(self):
        self.zero_xyz_cb()
        self.zero_rpy_cb()

    def zero_xyz_cb(self):
        self.x = self.initial_xyz[0]
        self.y = self.initial_xyz[1]
        self.z = self.initial_xyz[2]
        self.x_slider.set(self.x)
        self.y_slider.set(self.y)
        self.z_slider.set(self.z)

    def zero_rpy_cb(self):
        self.ro = self.initial_rpy[0]
        self.pi = self.initial_rpy[1]
        self.ya = self.initial_rpy[2]
        self.ro_slider.set(self.ro)
        self.pi_slider.set(self.pi)
        self.ya_slider.set(self.ya)

    def create_gui(self, app, obj_name):
        _width = 20
        _length = 300
        _resolution = 0.0001
        # Define Sliders and Labels

        row_count = 0
        obj_label = Label(app, text='CONTROLLING OBJECT: ' + obj_name, fg="Red")
        obj_label.grid(row=row_count, columnspan=2, pady=5)

        row_count = row_count + 1

        min_v = self.initial_xyz[0] - self.range_xyz / 2.0
        max_v = self.initial_xyz[0] + self.range_xyz / 2.0
        self.x_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=self.x_cb)
        self.x_slider.grid(row=row_count, column=1)

        self.x_slider.set(self.x)

        row_count = row_count + 1

        x_label = Label(app, text="x")
        x_label.grid(row=row_count, column=1, pady=5)

        row_count = row_count + 1

        min_v = self.initial_xyz[1] - self.range_xyz / 2.0
        max_v = self.initial_xyz[1] + self.range_xyz / 2.0
        self.y_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=self.y_cb)
        self.y_slider.grid(row=row_count, column=1)

        self.y_slider.set(self.y)

        row_count = row_count + 1

        y_label = Label(app, text="y")
        y_label.grid(row=row_count, column=1)

        row_count = row_count + 1
        min_v = self.initial_xyz[2] - self.range_xyz / 2.0
        max_v = self.initial_xyz[2] + self.range_xyz / 2.0
        self.z_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=self.z_cb)
        self.z_slider.grid(row=row_count, column=1)
        self.z_slider.set(self.z)

        row_count = row_count + 1

        z_label = Label(app, text="z")
        z_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_xyz = Button(app, width=_width, command=self.zero_xyz_cb)
        zero_xyz.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_xyz_label = Label(app, text="Reset Position")
        zero_xyz_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        min_v = self.initial_rpy[0] - self.range_rpy / 2.0
        max_v = self.initial_rpy[0] + self.range_rpy / 2.0
        self.ro_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.roll_cb)
        self.ro_slider.grid(row=row_count, column=1)
        self.ro_slider.set(self.ro)

        row_count = row_count + 1

        roll_label = Label(app, text="roll")
        roll_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        min_v = self.initial_rpy[1] - self.range_rpy / 2.0
        max_v = self.initial_rpy[1] + self.range_rpy / 2.0
        self.pi_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.pitch_cb)
        self.pi_slider.grid(row=row_count, column=1)
        self.pi_slider.set(self.pi)

        row_count = row_count + 1

        pitch_label = Label(app, text="pitch")
        pitch_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        min_v = self.initial_rpy[2] - self.range_rpy / 2.0
        max_v = self.initial_rpy[2] + self.range_rpy / 2.0
        self.ya_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.yaw_cb)
        self.ya_slider.grid(row=row_count, column=1)
        self.ya_slider.set(self.ya)

        row_count = row_count + 1

        yaw_label = Label(app, text="yaw")
        yaw_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_rpy = Button(app, width=_width, command=self.zero_rpy_cb)
        zero_rpy.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_rpy_label = Label(app, text="Reset Rotation")
        zero_rpy_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_all = Button(app, width=_width, command=self.zero_all_cb)
        zero_all.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_all_label = Label(app, text="Reset All")
        zero_all_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        self.gr_slider = Scale(app, from_=0, to=1.0, resolution=self.resolution, width=_width, length=_length,
                               orient=HORIZONTAL,
                               command=self.gr_cb)
        self.gr_slider.grid(row=row_count, column=1)
        self.gr_slider.set(0.5)

        row_count = row_count + 1

        gr_label = Label(app, text="Gripper")
        gr_label.grid(row=row_count, column=1)
