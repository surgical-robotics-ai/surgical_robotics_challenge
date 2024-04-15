import time
import os
import sys
from glob import glob
import rosbag
import gc
import numpy as np
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.units_conversion import SimToSI
dynamic_path = os.path.abspath(__file__ + "/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from surgical_robotics_challenge.utils.utilities import convert_mat_to_frame
from surgical_robotics_challenge.utils.utilities import convert_frame_to_mat
from scipy.spatial.transform import Rotation as R


def needle_msg_to_mtx(msg):
    pose_info = msg.pose
    pos = np.array([pose_info.position.x, pose_info.position.y, pose_info.position.z])
    ori_q = np.array([pose_info.orientation.x, pose_info.orientation.y, pose_info.orientation.z, pose_info.orientation.w])
    mtx = np.eye(4)
    rot = R.from_quat(ori_q)
    mtx[0:3, 3] = pos / SimToSI.linear_factor
    mtx[0:3, 0:3] = rot.as_matrix()
    return mtx

### this function is a duplicate of gripper_cb() [line #312] in mtm_device_crtk.py
def gripper_msg_to_jaw(msg):
    min = -0.1
    max = 0.51
    jaw_angle = msg.position[0] + min / (max - min)
    return jaw_angle

if __name__ == '__main__':
    data_folder = os.path.join(dynamic_path, "test_data")
    file_list = glob(os.path.join(data_folder, "*.bag"))
    rosbag_name = file_list[0]
    print('The name of the rosbag is: \n', rosbag_name)

    bag = rosbag.Bag(rosbag_name)
    topics = list(bag.get_type_and_topic_info()[1].keys())
    types = [val[0] for val in bag.get_type_and_topic_info()[1].values()]

    count = 0
    topics_name = []
    psm1_pos_ambf = []
    psm2_pos_ambf = []
    t_psm1 = []
    t_psm2 = []
    psm1_jaw_ambf = []
    psm2_jaw_ambf = []
    needle_pos = []

    ## ambf raw replay
    for topic, msg, t in bag.read_messages(topics=topics[14]):
        assert topic == '/ambf/env/psm1/baselink/State', 'load incorrect topics'
        psm1_pos_temp = [msg.joint_positions[0],
                         msg.joint_positions[1],
                         msg.joint_positions[2] / SimToSI.linear_factor,
                         msg.joint_positions[3],
                         msg.joint_positions[4],
                         msg.joint_positions[5]]
        psm1_pos_ambf.append(psm1_pos_temp)
        t_psm1.append(t)
        count += 1
    print('psm1 ambf record count: ', count)
    count = 0

    for topic, msg, t in bag.read_messages(topics=topics[15]):
        assert topic == '/ambf/env/psm2/baselink/State', 'load incorrect topics'
        psm2_pos_temp = [msg.joint_positions[0],
                         msg.joint_positions[1],
                         msg.joint_positions[2] / SimToSI.linear_factor,
                         msg.joint_positions[3],
                         msg.joint_positions[4],
                         msg.joint_positions[5]]
        psm2_pos_ambf.append(psm2_pos_temp)
        t_psm2.append(t)
        count += 1
    print('psm2 ambf record count: ', count)
    count = 0

    for topic, msg, t in bag.read_messages(topics=topics[0]):
        assert topic == '/MTML/gripper/measured_js', 'load incorrect topics'
        psm1_jaw_ambf_temp = gripper_msg_to_jaw(msg)
        psm1_jaw_ambf.append(psm1_jaw_ambf_temp)
        count += 1
    print('MTML gripper record count: ', count)
    count = 0

    for topic, msg, t in bag.read_messages(topics=topics[1]):
        assert topic == '/MTMR/gripper/measured_js', 'load incorrect topics'
        psm2_jaw_ambf_temp = gripper_msg_to_jaw(msg)
        psm2_jaw_ambf.append(psm2_jaw_ambf_temp)
        count += 1
    print('MTMR gripper record count: ', count)
    count = 0
    gc.collect()

    simulation_manager = SimulationManager('record_test')
    time.sleep(0.5)
    w = simulation_manager.get_world_handle()
    time.sleep(0.2)
    w.reset_bodies()
    time.sleep(0.2)
    cam = ECM(simulation_manager, 'CameraFrame')
    cam.servo_jp([0.0, 0.05, -0.01, 0.0])
    time.sleep(0.5)
    psm1 = PSM(simulation_manager, 'psm1', add_joint_errors=False)
    time.sleep(0.5)
    psm2 = PSM(simulation_manager, 'psm2', add_joint_errors=False)
    time.sleep(0.5)
    needle = simulation_manager.get_obj_handle('Needle')
    time.sleep(0.2)

    needle_pose_list = []

    total_num = min(len(psm1_pos_ambf), len(psm2_pos_ambf), len(psm1_jaw_ambf), len(psm2_jaw_ambf))
    print('Total number of elements : ',total_num)
    for i in range(total_num):
        # cam.servo_jp(ecm_pos[i])
        psm1.servo_jp(psm1_pos_ambf[i])
        psm1.set_jaw_angle(psm1_jaw_ambf[i])
        psm2.servo_jp(psm2_pos_ambf[i])
        psm2.set_jaw_angle(psm2_jaw_ambf[i])
        time.sleep(0.01)
        count += 1
        sys.stdout.write(f'\r Running progress {count}/{total_num}')
        sys.stdout.flush()

    print('Done')
