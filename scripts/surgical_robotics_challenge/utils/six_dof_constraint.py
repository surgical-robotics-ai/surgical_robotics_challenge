from ambf_client import Client
from PyKDL import Frame, Vector, Rotation
import numpy as np
import rospy
import time
import Tkinter


def activate_cb_lin():
    global activate_lin_constraint
    activate_lin_constraint = not activate_lin_constraint
    global activate_ang_constraint
    activate_ang_constraint = not activate_ang_constraint
    print("ACTIVATE CONSTRAINTS: ", activate_lin_constraint)


# def activate_cb_ang():
#     global activate_ang_constraint
#     activate_ang_constraint = not activate_ang_constraint
#     print("ACTIVATE ANGULAR CONSTRAINT: ", activate_ang_constraint)


def get_abs_vec(v):
    abs_vec = Vector(abs(v[0]), abs(v[1]), abs(v[2]))
    return abs_vec


def get_elementwise_max(a, b):
    m1 = max(a[0], b[0])
    m2 = max(a[1], b[1])
    m3 = max(a[2], b[2])
    return Vector(m1, m2, m3)


def get_vec_sign(v):
    s1 = np.sign(v[0])
    s2 = np.sign(v[1])
    s3 = np.sign(v[2])
    return Vector(s1, s2, s3)


def element_wise_mul(v1, v2):
    v = Vector(0, 0, 0)
    v[0] = v1[0] * v2[0]
    v[1] = v1[1] * v2[1]
    v[2] = v1[2] * v2[2]
    return v


def compute_lin_over_thresh(err, thres):
    error_sign = get_vec_sign(err)
    abs_error = get_abs_vec(err)
    net_thres = abs_error - thres
    over_thres = get_elementwise_max(net_thres, Vector(0, 0, 0))
    over_thres = element_wise_mul(error_sign, over_thres)
    return over_thres


def compute_lin_error(set_point, cur_pos):
    return set_point - cur_pos


def compute_ang_error(set_point, cur_pos):
    sp = Rotation.RPY(set_point[0], set_point[1], set_point[2])
    dr = cur_pos.Inverse() * sp
    ang, axis = dr.GetRotAngle()
    dr = cur_pos * axis * ang
    return dr


def get_box_pos():
    bp = box.get_pos()
    return Vector(bp.x, bp.y, bp.z)


def get_box_rot():
    br = box.get_rot()
    return Rotation.Quaternion(br.x, br.y, br.z, br.w)


tk = Tkinter.Tk()
tk.title("Constraint")
tk.geometry("250x250")
activate_button_lin = Tkinter.Button(tk, text="Activate", command=activate_cb_lin, height=3, width=50, bg="red")

activate_button_lin.pack()

c = Client('six_dof_constraint_test')
c.connect()

box = c.get_obj_handle('Cube')
time.sleep(0.5)

# Linear
constraint_lin_pos = Vector(0, 0, 0)
lin_thresh = Vector(1.0, 1.0, 1.0) * 0.001
lin_P_gains = 10.0
lin_D_gains = 0.0
last_lin_error = Vector(0, 0, 0)
lin_error = Vector(0, 0, 0)
activate_lin_constraint = False
lin_proximity_trigger = False

# Angular
constraint_ang_pos = Vector(0, 0, 0)
ang_thresh = Vector(1.0, 1.0, 1.0) * 0.05
ang_P_gains = 5.0
ang_D_gains = 0.0
last_ang_error = Vector(0, 0, 0)
ang_error = Vector(0, 0, 0)
activate_ang_constraint = False
ang_proximity_trigger = False

last_time = time.time()
cur_time = time.time()
rate = rospy.Rate(1000)

while not rospy.is_shutdown():
    # Linear
    box_pos = get_box_pos()
    last_lin_error = lin_error
    lin_error = compute_lin_error(constraint_lin_pos, box_pos)

    if lin_error.Norm() < 0.05 and lin_proximity_trigger is False:
        # Activate after the first time the block is near the constraint
        lin_proximity_trigger = True
        print("ACTIVATING LINEAR CONSTRAINT AT ERROR MAG: ", lin_error.Norm())

    if lin_proximity_trigger:
        lin_over_thresh = compute_lin_over_thresh(lin_error, lin_thresh)
        constraint_lin_pos = constraint_lin_pos - lin_over_thresh
        # print(constraint_lin_pos, lin_over_thresh)

    # Angular
    box_rot = get_box_rot()
    last_ang_error = ang_error
    ang_error = compute_ang_error(constraint_ang_pos, box_rot)

    if ang_error.Norm() < 0.05 and ang_proximity_trigger is False:
        # Activate after the first time the block is near the constraint
        ang_proximity_trigger = True
        print("ACTIVATING ANGULAR CONSTRAINT AT ERROR MAG: ", ang_error.Norm())

    if ang_proximity_trigger:
        ang_over_thresh = compute_lin_over_thresh(ang_error, ang_thresh)
        constraint_ang_pos = constraint_ang_pos - ang_over_thresh
        # print(constraint_pos, over_thresh)

    last_time = cur_time
    cur_time = time.time()
    dt = min(cur_time - last_time, 0.001)

    if activate_lin_constraint:
        lin_cmd = lin_P_gains * lin_error + lin_D_gains * (lin_error - last_lin_error) / dt
        box.set_linear_vel(lin_cmd[0], lin_cmd[1], lin_cmd[2])
        # box.set_force(lin_cmd[0], lin_cmd[1], lin_cmd[2])

        ang_cmd = ang_P_gains * ang_error + ang_D_gains * (ang_error - last_ang_error) / dt
        box.set_angular_vel(ang_cmd[0], ang_cmd[1], ang_cmd[2])
        # box.set_torque(ang_cmd[0], ang_cmd[1], ang_cmd[2])
    else:
        box.set_force(0.0, 0.0, 0.0)
        box.set_torque(0, 0, 0)

    rate.sleep()
    tk.update()
