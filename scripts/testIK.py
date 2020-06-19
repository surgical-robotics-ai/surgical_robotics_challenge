from psmIK import *
from ambf_client import Client
import time
import rospy
from Tkinter import *
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser

class JawControlGUI:
    def __init__(self):
        self.App = Tk()
        self.min = 0.0
        self.max = 1.0
        self.resolution = 0.001
        self.slider1_val = 0.0
        self.slider2_val = 0.0

        self.jaw1_slider = Scale(self.App, from_=self.min, to=self.max, resolution=self.resolution,
                                 orient=HORIZONTAL, command=self.slider1_cb)
        self.jaw1_slider.grid(row=0, column=0)

        self.jaw1_slider = Scale(self.App, from_=self.min, to=self.max, resolution=self.resolution,
                                 orient=HORIZONTAL, command=self.slider2_cb)
        self.jaw1_slider.grid(row=1, column=0)

    def slider1_cb(self, val):
        self.slider1_val = float(val)

    def slider2_cb(self, val):
        self.slider2_val = float(val)


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


def test_ambf_psm(run_psm_one, run_psm_two):
    c = Client()
    c.connect()

    base1 = None
    base2 = None
    target1 = None
    target2 = None

    if run_psm_one is True:
        print('PREPARING TO LOAD IK FOR PSM1')
        base1 = c.get_obj_handle('psm1/baselink')
        target1 = c.get_obj_handle('psm1/target')

    if run_psm_two is True:
        print('PREPARING TO LOAD IK FOR PSM2')
        base2 = c.get_obj_handle('psm2/baselink')
        target2 = c.get_obj_handle('psm2/target')

    if not run_psm_one and not run_psm_two:
        print('YOU HAVE TO RUN ATLEAST ONE PSMS IK FOR THIS SCRIPT TO DO ANYTHING')
        return False
    else:
        jaw_ctrl = JawControlGUI()

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
        jaw_ctrl.App.update()

        if run_psm_one is True:
            jaw1_val = jaw_ctrl.slider1_val
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
            base1.set_joint_pos('toolpitchlink-toolgripper1link', -computed_q[5]+jaw1_val)
            base1.set_joint_pos('toolpitchlink-toolgripper2link', computed_q[5]+jaw1_val)

        if run_psm_two is True:
            jaw2_val = jaw_ctrl.slider2_val
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
            base2.set_joint_pos('toolpitchlink-toolgripper1link', -computed_q[5]+jaw2_val)
            base2.set_joint_pos('toolpitchlink-toolgripper2link', computed_q[5]+jaw2_val)

        time.sleep(0.05)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print parsed_args

    test_ambf_psm(parsed_args.run_psm_one, parsed_args.run_psm_two)
