import copy
import numpy
from PyKDL import Vector, Rotation, Frame
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
import rospy
import sys
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.kinematics.psmKinematics import PSMKinematicSolver
from surgical_robotics_challenge.utils.utilities import convert_mat_to_frame
from surgical_robotics_challenge.utils.utilities import convert_frame_to_mat
from utilities import cartesian_interpolate_step
import os
import sys
dynamic_path = os.path.abspath(__file__+"/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)

def pykdl_to_np(T_pykdl: Frame) -> np.matrix:
    rot_des = R.from_quat(T_pykdl.M.GetQuaternion()).as_matrix()
    pos_des = np.array([T_pykdl.p.x(), T_pykdl.p.y(), T_pykdl.p.z()])
    T_np = np.eye(4)
    T_np[0:3, 0:3] = rot_des
    T_np[0:3, 3] = pos_des
    return T_np

def np_to_pykdl(T_np: np.matrix) -> Frame:
    rot_des = np.squeeze(np.array(T_np[0:3, 0:3].reshape(-1, 1))).tolist()
    pos_des = np.squeeze(np.array(T_np[0:3, 3])).tolist()
    v = Vector(pos_des[0], pos_des[1], pos_des[2])
    r = Rotation(rot_des[0], rot_des[1], rot_des[2],
                 rot_des[3], rot_des[4], rot_des[5],
                 rot_des[6], rot_des[7], rot_des[8])
    T_pydkl = Frame(r, v)
    return T_pydkl


def gripper_to_yaw(T: Frame) -> Frame:
    L_yaw2ctrlpnt = 0.0
    offset_gripper = Frame(Rotation.RPY(0, 0, 0),
                           L_yaw2ctrlpnt * Vector(0.0, 0.0, -1.0))
    offset_x = Frame(Rotation.RPY(PI_2, 0, 0), Vector(0.0, 0.0, 0.0))
    offset_y = Frame(Rotation.RPY(0, - PI_2, 0), Vector(0.0, 0.0, 0.0))
    T_out = T * offset_gripper * offset_x * offset_y
    return T_out

def yaw_to_gripper(T: Frame) -> Frame:
    L_yaw2ctrlpnt = 0.0
    offset_gripper = Frame(Rotation.RPY(0, 0, 0),
                           L_yaw2ctrlpnt * Vector(0.0, 0.0, 1.0))
    offset_x = Frame(Rotation.RPY(-np.pi/2, 0, 0), Vector(0.0, 0.0, 0.0))
    offset_y = Frame(Rotation.RPY(0, np.pi/2, 0), Vector(0.0, 0.0, 0.0))
    T_out =  T* offset_y * offset_x * offset_gripper
    return T_out



if __name__ == "__main__":
    simulation_manager = SimulationManager('grasp_needle')
    time.sleep(0.2)
    w = simulation_manager.get_world_handle()
    time.sleep(0.2)
    w.reset_bodies()
    time.sleep(0.2)

    # set tool id
    # tool_id = 400006 # the tool id for the simple psm
    tool_id = 420006 # the tool id for the real psm
    psm_ks = PSMKinematicSolver(psm_type=tool_id, tool_id=tool_id)

    cam = ECM(simulation_manager, "CameraFrame")
    cam.servo_jp([0.0, 0.05, -0.01, 0.0])
    time.sleep(0.2)
    psm1 = PSM(simulation_manager, "psm1", add_joint_errors=False, tool_id=tool_id_simple)
    time.sleep(0.2)
    psm2 = PSM(simulation_manager, "psm2", add_joint_errors=False, tool_id=tool_id_simple)
    time.sleep(0.2)
    needle = simulation_manager.get_obj_handle('Needle')
    time.sleep(0.2)

    grasp_arm = 'psm2'
    # grasp_arm = 'psm1'

    # init joint position from the recording of old phantom
    psm1_init = [0.30780306382205863, -0.22222915389237488, 0.1423643360325034,
                 -1.3613186165319513,0.5750600725456388, -0.8399263308008617]
    psm2_init = [-0.46695894800579796, -0.17860657808832947, 0.15012366098379068,
                 -1.0873261421084663, 0.7172512403887915, 0.48780102579228307]

    ## offset of PSM
    offset_psm1 = Frame(Rotation.RPY(-np.pi / 2., np.pi*1/3, 0.),
                        Vector(0.009973019361495972, -0.005215135216712952, 0.003237169608473778))

    offset_psm2 = Frame(Rotation.RPY(-np.pi / 2., 0., 0.),
                        Vector(0.009973019361495972, -0.005215135216712952, 0.003237169608473778))
    if grasp_arm == 'psm1':
        psm1.move_jp(psm1_init)
        psm1.set_jaw_angle(0.7)
    else:
        psm2.move_jp(psm2_init)
        psm2.set_jaw_angle(0.7)
    time.sleep(3.0)

    if grasp_arm == 'psm2':
        ### for psm 2
        T_psm_w_b = psm2.get_T_b_w()  # from world to PSM base
        T_psm_b_w = psm2.get_T_w_b()  # from PSM base to world
        psm2_pose_cp = psm2.measured_cp()
        psm2_pose = psm2.measured_jp()
        psm2_pose.append(0.0)
        mtx_tool_tip = psm_ks.compute_FK(psm2_pose, 7) ### FK to the tool tip
        mtx_tool_yaw = psm_ks.compute_FK(psm2_pose, 6) ### FK to the tool yaw link
        T_psm_yaw = convert_mat_to_frame(mtx_tool_yaw)
    else:
        #### for psm 1
        T_psm_w_b = psm1.get_T_b_w()  # from world to PSM base
        T_psm_b_w = psm1.get_T_w_b()  # from PSM base to world
        psm1_pose_cp = psm1.measured_cp()
        psm1_pose = psm1.measured_jp()
        psm1_pose.append(0.0)
        mtx_tool_tip = psm_ks.compute_FK(psm1_pose, 7)  ### FK to the tool tip
        mtx_tool_yaw = psm_ks.compute_FK(psm1_pose, 6)  ### FK to the tool yaw link
        T_psm_yaw = convert_mat_to_frame(mtx_tool_yaw)



    T_offset_w = Frame(Rotation.RPY(np.pi, 0.0, 0),
                             Vector(0.0, 0.0, 0.0))

    T_psmyaw_w = T_psm_w_b * T_psm_yaw * T_offset_w ## == TtInw
    # ##### attach needle #######

    if grasp_arm == 'psm2':
        T_needle_psmtip = offset_psm2
    else:
        T_needle_psmtip = offset_psm1

    time.sleep(0.5)
    done = False
    T_nINw = needle.get_pose() ################################## replace this line ###############################
    T_tINw = copy.deepcopy(T_psmyaw_w)
    print('Move to the desired pose')
    while not done:
        T_tINw_cmd = T_nINw * T_needle_psmtip.Inverse()
        T_delta, done = cartesian_interpolate_step(T_tINw, T_tINw_cmd, 0.01, 0.005)
        r_delta = T_delta.M.GetRPY()
        T_cmd = Frame()
        T_cmd.p = T_tINw.p + T_delta.p
        T_cmd.M = T_tINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
        T_tINw = T_cmd
        T_move = yaw_to_gripper(T_psm_b_w * T_cmd * T_offset_w.Inverse())
        if grasp_arm == 'psm2':
            ### for psm 2
            psm2.servo_cp(T_move)
            psm2.set_jaw_angle(0.5)
        else:
            ### for psm 1
            psm1.servo_cp(T_move)
            psm1.set_jaw_angle(0.5)
        time.sleep(0.01)
    print("Done")
    time.sleep(1.0)
    if grasp_arm == 'psm2':
        psm2.set_jaw_angle(0.0)
    else:
        psm1.set_jaw_angle(0.0)
    time.sleep(1.0)

    # #### Open the jaws if required
    # psm2.set_jaw_angle(0.7)
    # psm1.set_jaw_angle(0.7)
    # time.sleep(2.0)


