from psmIK import *
from ambf_client import Client
import time
import rospy
from PyKDL import Frame, Rotation, Vector


def test_ik():
    # We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
    # in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)
    test_q = [-0.3, 0.2, 0.1, -0.9, 0.0, 0.0, 0.0]
    T_7_0 = compute_FK(test_q)
    computed_q = compute_IK(convert_mat_to_frame(T_7_0))
    for i in range(0, 6):
        computed_q[i] = round(computed_q[i], 4)

    print('Test Q: ', test_q[0:6])
    print('Comp Q: ', computed_q)


def test_ambf_psm():
    c = Client()
    c.connect()

    base1 = c.get_obj_handle('psm1/baselink')
    base2 = c.get_obj_handle('psm2/baselink')
    target1 = c.get_obj_handle('psm1/target')
    target2 = c.get_obj_handle('psm2/target')

    time.sleep(1.0)

    # The following are the names of the controllable joints.
    #  'baselink-yawlink', 0
    #  'yawlink-pitchbacklink', 1
    #  'pitchendlink-maininsertionlink', 2
    #  'maininsertionlink-toolrolllink', 3
    #  'toolrolllink-toolpitchlink', 4
    #  'toolpitchlink-toolgripper1link', 5a
    #  'toolpitchlink-toolgripper2link', 5b

    while not rospy.is_shutdown():
        p = base1.get_pos()
        q = base1.get_rot()
        P_b_w = Vector(p.x, p.y, p.z)
        R_b_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
        T_b_w = Frame(R_b_w, P_b_w)
        p = target1.get_pos()
        q = target1.get_rot()
        P_t_w = Vector(p.x, p.y, p.z)
        R_t_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
        T_t_w = Frame(R_t_w, P_t_w)
        T_t_b = T_b_w.Inverse() * T_t_w
        computed_q = compute_IK(T_t_b)

        # print('SETTING JOINTS: ')
        # print(computed_q)

        base1.set_joint_pos('baselink-yawlink', computed_q[0])
        base1.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
        base1.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
        base1.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
        base1.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
        base1.set_joint_pos('toolpitchlink-toolgripper1link', -computed_q[5])
        base1.set_joint_pos('toolpitchlink-toolgripper2link', computed_q[5])

        p = base2.get_pos()
        q = base2.get_rot()
        P_b_w = Vector(p.x, p.y, p.z)
        R_b_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
        T_b_w = Frame(R_b_w, P_b_w)
        p = target2.get_pos()
        q = target2.get_rot()
        P_t_w = Vector(p.x, p.y, p.z)
        R_t_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
        T_t_w = Frame(R_t_w, P_t_w)
        T_t_b = T_b_w.Inverse() * T_t_w
        computed_q = compute_IK(T_t_b)

        # print('SETTING JOINTS: ')
        # print(computed_q)

        base2.set_joint_pos('baselink-yawlink', computed_q[0])
        base2.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
        base2.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
        base2.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
        base2.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
        base2.set_joint_pos('toolpitchlink-toolgripper1link', -computed_q[5])
        base2.set_joint_pos('toolpitchlink-toolgripper2link', computed_q[5])

        time.sleep(0.05)


if __name__ == "__main__":
    # test_ik()
    test_ambf_psm()
