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
    
    # Base in Needle Origin
    T_base_origin = Frame(Rotation.RPY(0., 0., 0.), Vector(-0.009513319469988346, 0.0010262969881296158, 0.0))
    # Mid in Needle Origin
    T_mid_origin = Frame(Rotation.RPY(0., 0., -1.091), Vector(-0.004098702222108841, 0.008965075016021729, 0.0))
    # Tip in Needle Origin
    T_tip_origin = Frame(Rotation.RPY(0., 0., -0.688), Vector(0.00661780871450901, 0.0069734565913677216, 0.0))


class TeleopScale:
    scale_factor = 0.005