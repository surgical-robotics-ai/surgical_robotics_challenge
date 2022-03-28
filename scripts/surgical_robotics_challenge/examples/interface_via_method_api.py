# Import the relevant classes

from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.scene import Scene

from PyKDL import Frame, Rotation, Vector
import numpy as np

# Import AMBF Client
from ambf_client import Client
import time

def add_break(s):
    time.sleep(s)
    print('-------------')

# Create an instance of the client
my_client = Client('my_example_client')
my_client.connect()
time.sleep(0.5)
world_handle = my_client.get_world_handle()

# Get a handle to PSM1
psm1 = PSM(my_client, 'psm1')
# Get a handle  to PSM2
psm2 = PSM(my_client, 'psm2')
# Get a handle to ECM
ecm = ECM(my_client, 'CameraFrame')
# Get a handle to scene to access its elements, i.e. needle and entry / exit points
scene = Scene(my_client)
# Small sleep to let the handles initialize properly
time.sleep(0.5)

print("Resetting the world")
world_handle.reset()
add_break(3.0)

# To get the pose of objects
print("PSM1 End-effector pose in Base Frame", psm1.measured_cp())
print("PSM1 Base pose in World Frmae", psm1.get_T_b_w())
print("PSM1 Joint state", psm1.measured_jp())
add_break(1.0)
print("PSM2 End-effector pose in Base Frame", psm2.measured_cp())
print("PSM2 Base pose in World Frmae", psm2.get_T_b_w())
print("PSM2 Joint state", psm2.measured_jp())
add_break(1.0)
# Things are slightly different for ECM as the `measure_cp` returns pose in the world frame
print("ECM pose in World", ecm.measured_cp())
add_break(1.0)
# Scene object poses are all w.r.t World
print("Entry 1 pose in World", scene.entry1_measured_cp())
print("Exit 4 pose in World", scene.exit4_measured_cp())
add_break(1.0)
####
#### Your control / ML - RL Code will go somewhere in this script
####

# The PSMs can be controlled either in joint space or cartesian space. For the
# latter, the `servo_cp` command sets the end-effector pose w.r.t its Base frame.
T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/2.), Vector(0., 0., -0.8))
print("Setting the end-effector frame of PSM1 w.r.t Base", T_e_b)
psm1.servo_cp(T_e_b)
add_break(1.0)
T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/4.), Vector(0.1, -0.1, -0.8))
print("Setting the end-effector frame of PSM2 w.r.t Base", T_e_b)
psm2.servo_cp(T_e_b)
add_break(1.0)
# Controlling in joint space
jp = [0., 0., 1.0, 0.5, 0.7, 0.9]
print("Setting PSM1 joint positions to ", jp)
psm1.servo_jp(jp)
add_break(1.0)
jp = [0., 0., 1.0, -0.5, -0.7, -0.9]
print("Setting PSM2 joint positions to ", jp)
psm2.servo_jp(jp)
add_break(1.0)
# The ECM should always be controlled using its joint interface
jp = [0., 0., 0.2, 0.3]
print("Setting ECM joint positions to ", jp)
ecm.servo_jp(jp)
add_break(1.0)

print("Resetting the world")
world_handle.reset()
add_break(3.0)
print('END')
