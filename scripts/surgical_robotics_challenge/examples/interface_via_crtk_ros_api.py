# Import the relevant classes
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from PyKDL import Frame, Rotation, Vector
import time
from enum import Enum
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from surgical_robotics_challenge.task_completion_report import TaskCompletionReport
from surgical_robotics_challenge.utils.utilities import *


def add_break(s):
    time.sleep(s)
    print('-------------')


class ImageSub:
    def __init__(self, image_topic):
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_cb)
        self.image_msg = Image()

    def image_cb(self, image_msg):
        self.image_msg = image_msg


class ArmType(Enum):
    PSM1=1
    PSM2=2
    ECM=3


def list_to_sensor_msg_position(jp_list):
    msg = JointState()
    msg.position = jp_list
    return msg


class ARMInterface:
    def __init__(self, arm_type):
        if arm_type == ArmType.PSM1:
            arm_name = '/CRTK/psm1'
        elif arm_type == ArmType.PSM2:
            arm_name = '/CRTK/psm2'
        elif arm_type == ArmType.ECM:
            arm_name = '/CRTK/ecm'
        else:
            raise ("Error! Invalid Arm Type")

        self._cp_sub = rospy.Subscriber(arm_name + "/measured_cp", PoseStamped, self.cp_cb, queue_size=1)
        self._T_b_w_sub = rospy.Subscriber(arm_name + "/T_b_w", PoseStamped, self.T_b_w_cb, queue_size=1)
        self._jp_sub = rospy.Subscriber(arm_name + "/measured_js", JointState, self.jp_cb, queue_size=1)
        self.cp_pub = rospy.Publisher(arm_name + "/servo_cp", PoseStamped, queue_size=1)
        self.jp_pub = rospy.Publisher(arm_name + "/servo_jp", JointState, queue_size=1)
        self.jaw_jp_pub = rospy.Publisher(arm_name + '/jaw/' + 'servo_jp', JointState, queue_size=1)

        self.measured_cp_msg = None
        self.T_b_w_msg = None
        self.measured_jp_msg = None

    def cp_cb(self, msg):
        self.measured_cp_msg = msg

    def T_b_w_cb(self, msg):
        self.T_b_w_msg = msg

    def jp_cb(self, msg):
        self.measured_jp_msg = msg

    def measured_cp(self):
        return self.measured_cp_msg

    def get_T_b_w(self):
        return self.T_b_w_msg

    def measured_jp(self):
        return self.measured_jp_msg

    def servo_cp(self, pose):
        if type(pose) == Frame:
            msg = frame_to_pose_stamped(pose)
        else:
            msg = pose
        self.cp_pub.publish(msg)

    def servo_jp(self, jp):
        if type(jp) == list:
            msg = list_to_sensor_msg_position(jp)
        else:
            msg = jp
        self.jp_pub.publish(msg)

    def set_jaw_angle(self, val):
        msg = list_to_sensor_msg_position([val])
        self.jaw_jp_pub.publish(msg)


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


class SceneInterface:
    def __init__(self):
        self._scene_object_poses = dict()
        self._scene_object_poses[SceneObjectType.Needle] = None
        self._scene_object_poses[SceneObjectType.Entry1] = None
        self._scene_object_poses[SceneObjectType.Entry2] = None
        self._scene_object_poses[SceneObjectType.Entry3] = None
        self._scene_object_poses[SceneObjectType.Entry4] = None
        self._scene_object_poses[SceneObjectType.Exit1] = None
        self._scene_object_poses[SceneObjectType.Exit2] = None
        self._scene_object_poses[SceneObjectType.Exit3] = None
        self._scene_object_poses[SceneObjectType.Exit4] = None
        self._subs = []

        namespace = '/CRTK/'
        suffix = '/measured_cp'
        for k, i in self._scene_object_poses.items():
            self._subs.append(rospy.Subscriber(namespace + k.name + suffix, PoseStamped,
                                               self.state_cb, callback_args=k, queue_size=1))

        self._task_3_ready = False
        self._task_3_setup_init_pub = rospy.Publisher('/CRTK/scene/task_3_setup/init', Empty, queue_size=1)

        self._task_3_setup_ready_sub = rospy.Subscriber('/CRTK/scene/task_3_setup/ready',
                                                        Empty, self.task_3_setup_ready_cb, queue_size=1)

    def state_cb(self, msg, key):
        self._scene_object_poses[key] = msg

    def measured_cp(self, object_type):
        return self._scene_object_poses[object_type]

    def task_3_setup_ready_cb(self, msg):
        self._task_3_ready = True

    def task_3_setup_init(self):
        self._task_3_ready = False
        self._task_3_setup_init_pub.publish(Empty())
        while not self._task_3_ready:
            time.sleep(0.1)


class WorldInterface:
    def __init__(self):
        self._reset_world_pub = rospy.Publisher('/ambf/env/World/Command/Reset', Empty, queue_size=1)
        self._reset_bodies_pub = rospy.Publisher('/ambf/env/World/Command/ResetBodies', Empty, queue_size=1)

    def reset(self):
        self._reset_world_pub.publish(Empty())

    def reset_bodies(self):
        self._reset_bodies_pub.publish(Empty())


# Create an instance of the client
rospy.init_node('your_name_node')
time.sleep(0.5)
world_handle = WorldInterface()

# Get a handle to PSM1
psm1 = ARMInterface(ArmType.PSM1)
# Get a handle  to PSM2
psm2 = ARMInterface(ArmType.PSM2)
# Get a handle to ECM
ecm = ARMInterface(ArmType.ECM)
# Get a handle to scene to access its elements, i.e. needle and entry / exit points
scene = SceneInterface()
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
scene.task_3_setup_init()

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
print("Entry 1 pose in World", scene.measured_cp(SceneObjectType.Entry1))
print("Exit 4 pose in World", scene.measured_cp(SceneObjectType.Exit4))
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
