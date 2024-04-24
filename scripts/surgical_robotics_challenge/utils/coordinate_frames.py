from PyKDL import Vector, Frame, Rotation
import numpy as np


class ECM:
    T_t_b_home = Frame(Rotation.RPY(np.pi, 0.0, np.pi/2.), Vector(0.0, 0.0, -1.0))


class PSM:
    T_t_b_home = Frame(Rotation.RPY(np.pi, 0.0, np.pi/2.), Vector(0.0, 0.0, -1.0))


class PSM1(PSM):
    T_tip_cam = Frame(Rotation.RPY(np.pi, 0.0, -np.pi/2.), Vector(-0.02, 0.0, -0.1))


class PSM2(PSM):
    T_tip_cam = Frame(Rotation.RPY(np.pi, 0.0, -np.pi/2.), Vector(0.02, 0.0, -0.1))


class PSM3(PSM):
    T_tip_cam = Frame(Rotation.RPY(np.pi, 0.0, -np.pi/2.), Vector(0.02, 0.0, -0.1))


class Needle:
    T_center_psmtip = Frame(Rotation.RPY(-np.pi / 2., 0., 0.),
                            Vector(0.009973019361495972, -0.005215135216712952, 0.003237169608473778))


class TeleopScale:
    scale_factor = 0.005