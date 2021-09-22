import numpy as np
from enum import Enum


class JointType(Enum):
    REVOLUTE = 0
    PRISMATIC = 1


class Convention(Enum):
    STANDARD = 0
    MODIFIED = 1


class DH:
    def __init__(self, alpha, a, theta, d, offset, joint_type, convention):
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d
        self.offset = offset
        self.joint_type = joint_type
        self.convention = convention

    def mat_from_dh(self, alpha, a, theta, d, offset, joint_type, convention):
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        th = 0.0
        if joint_type == JointType.REVOLUTE:
            th = theta + offset
        elif joint_type == JointType.PRISMATIC:
            d = d + offset + theta
        else:
            assert joint_type == JointType.REVOLUTE and joint_type == JointType.PRISMATIC
            return

        ct = np.cos(th)
        st = np.sin(th)

        if convention == Convention.STANDARD:
            mat = np.mat([
                [ct, -st * ca,  st * sa,  a * ct],
                [st,  ct * ca, -ct * sa,  a * st],
                [0,        sa,       ca,       d],
                [0,         0,        0,       1]
            ])
        elif convention == Convention.MODIFIED:
            mat = np.mat([
                [ct, -st, 0, a],
                [st * ca, ct * ca, -sa, -d * sa],
                [st * sa, ct * sa, ca, d * ca],
                [0, 0, 0, 1]
            ])
        else:
            raise 'ERROR, DH CONVENTION NOT UNDERSTOOD'

        return mat

    def get_trans(self):
        return self.mat_from_dh(self.alpha, self.a, self.theta, self.d, self.offset, self.joint_type, self.convention)


def enforce_limits(j_raw, joint_lims):
    num_joints = len(j_raw)
    j_limited = [0.0]*num_joints

    for idx in range(num_joints):
        min_lim = joint_lims[idx][0]
        max_lim = joint_lims[idx][1]
        j_limited[idx] = max(min_lim, min(j_raw[idx], max_lim))

    return j_limited