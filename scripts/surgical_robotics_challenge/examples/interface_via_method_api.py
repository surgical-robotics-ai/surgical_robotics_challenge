# Import the relevant classes

from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.scene import Scene
import rospy
from sensor_msgs.msg import Image
from surgical_robotics_challenge.task_completion_report import TaskCompletionReport, PoseStamped
from surgical_robotics_challenge.kinematics.psmKinematics import ToolType

from PyKDL import Frame, Rotation, Vector
import numpy as np

# Import AMBF Client
from surgical_robotics_challenge.simulation_manager import SimulationManager
import time


def add_break(s):
    time.sleep(s)
    print('-------------')


class ImageSub:
    def __init__(self, image_topic):
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_cb)
        self.image_msg = Image()

    def image_cb(self, image_msg):
        self.image_msg = image_msg


# Create an instance of the client
simulation_manager = SimulationManager('my_example_client')
time.sleep(0.5)
world_handle = simulation_manager.get_world_handle()
tool_id = ToolType.Default

# Get a handle to PSM1
psm1 = PSM(simulation_manager, 'psm1', tool_id=tool_id)
# Get a handle  to PSM2
psm2 = PSM(simulation_manager, 'psm2', tool_id=tool_id)
# Get a handle to ECM
ecm = ECM(simulation_manager, 'CameraFrame')
# Get a handle to scene to access its elements, i.e. needle and entry / exit points
scene = Scene(simulation_manager)
# Create an instance of task completion report with you team name
task_report = TaskCompletionReport(team_name='my_team_name')
# Small sleep to let the handles initialize properly
add_break(0.5)

# Add you camera stream subs
cameraL_sub = ImageSub('/ambf/env/cameras/cameraL/ImageData')
cameraR_sub = ImageSub('/ambf/env/cameras/cameraR/ImageData')

print("Resetting the world")
world_handle.reset()
add_break(3.0)

####
# Your control / ML / RL Code will go somewhere in this script
####

# The PSMs can be controlled either in joint space or cartesian space. For the
# latter, the `servo_cp` command sets the end-effector pose w.r.t its Base frame.
T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/2.), Vector(0., 0., -0.13))
print("Setting the end-effector frame of PSM1 w.r.t Base", T_e_b)
psm1.servo_cp(T_e_b)
psm1.set_jaw_angle(0.2)
add_break(1.0)
T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/4.), Vector(0.01, -0.01, -0.13))
print("Setting the end-effector frame of PSM2 w.r.t Base", T_e_b)
psm2.servo_cp(T_e_b)
psm2.set_jaw_angle(0.5)
add_break(1.0)
# Controlling in joint space
jp = [0., 0., 0.135, 0.2, 0.3, 0.2]
print("Setting PSM1 joint positions to ", jp)
psm1.servo_jp(jp)
add_break(1.0)
jp = [0., 0., 0.135, -0.2, -0.3, -0.2]
print("Setting PSM2 joint positions to ", jp)
psm2.servo_jp(jp)
add_break(1.0)
# The ECM should always be controlled using its joint interface
jp = [0., 0.2, -0.03, 0.2]
print("Setting ECM joint positions to ", jp)
ecm.servo_jp(jp)
add_break(5.0)

# Set the initial conditions for task 3
scene.task_3_setup_init(psm2)

# To get the pose of objects
print("PSM1 End-effector pose in Base Frame", psm1.measured_cp())
print("PSM1 Base pose in World Frame", psm1.get_T_b_w())
print("PSM1 Joint state", psm1.measured_jp())
add_break(1.0)
print("PSM2 End-effector pose in Base Frame", psm2.measured_cp())
print("PSM2 Base pose in World Frame", psm2.get_T_b_w())
print("PSM2 Joint state", psm2.measured_jp())
add_break(1.0)
# Things are slightly different for ECM as the `measure_cp` returns pose in the world frame
print("ECM pose in World", ecm.measured_cp())
add_break(1.0)
# Scene object poses are all w.r.t World
print("Entry 1 pose in World", scene.entry1_measured_cp())
print("Exit 4 pose in World", scene.exit4_measured_cp())
add_break(1.0)

# When you are done with each task, you can report your results.
my_found_needle_pose = PoseStamped()
my_found_needle_pose.pose.orientation.w = 1.0
my_found_needle_pose.header.frame_id = 'CameraFrame'
task_report.task_1_report(my_found_needle_pose)

# For task 2, report when you think you are done
task_report.task_2_report(complete=True)

# For task 3, report when you think you are done
task_report.task_3_report(complete=True)

# Query Image Subs
print('cameraL Image Data Size: ', cameraL_sub.image_msg.height, cameraL_sub.image_msg.width)
print('cameraR Image Data Size: ', cameraR_sub.image_msg.height, cameraR_sub.image_msg.width)

# Reset ECM Back to Start
print("Resetting ECM pose")
ecm.servo_jp([0., 0., 0., 0.])
add_break(1.0)

# Open the jaw angle to drop the needle
psm2.set_jaw_angle(0.8)
add_break(1.0)

print('END')
