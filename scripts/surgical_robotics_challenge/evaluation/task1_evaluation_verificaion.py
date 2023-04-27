import time

from ambf_client import Client
from PyKDL import Frame, Rotation, Vector
import rospy
from geometry_msgs.msg import PoseStamped
from surgical_robotics_challenge import units_conversion
from surgical_robotics_challenge.utils.utilities import *


c = Client('eval_test')
c.connect()
time.sleep(0.5)
n = c.get_obj_handle('Needle')
e = c.get_obj_handle('CameraFrame')
time.sleep(0.5)
T_n_w = units_conversion.get_pose(n)
T_e_w = units_conversion.get_pose(e)
T_n_e = T_e_w.Inverse() * T_n_w
T_n_e.M.GetQuaternion()

m = frame_to_pose_stamped(T_n_e)
rep1 = rospy.Publisher('/surgical_robotics_challenge/completion_report/lola/task1', PoseStamped, queue_size=1)
time.sleep(0.5)
rep1.publish(m)
