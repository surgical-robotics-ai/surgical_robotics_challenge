from PyKDL import Vector, Rotation, Frame
from surgical_robotics_challenge.kinematics.DH import JointType


class SimToSI:
    linear_factor = 1.0
    angular_factor = 1.0


# Globals to convert between units
def get_pos(ambf_obj):
    v = Vector(ambf_obj.get_pos().x, ambf_obj.get_pos().y, ambf_obj.get_pos().z)
    return v / SimToSI.linear_factor


def get_rotation(ambf_obj):
    return Rotation.RPY(ambf_obj.get_rpy()[0] / SimToSI.angular_factor,
                        ambf_obj.get_rpy()[1] / SimToSI.angular_factor,
                        ambf_obj.get_rpy()[2] / SimToSI.angular_factor)


def get_pose(ambf_obj):
    return Frame(get_rotation(ambf_obj), get_pos(ambf_obj))


def set_pos(ambf_obj, pos):
    pos = pos * SimToSI.linear_factor
    ambf_obj.set_pos(pos[0], pos[1], pos[2])


def set_rpy(ambf_obj, r, p, y):
    r = r * SimToSI.angular_factor
    p = p * SimToSI.angular_factor
    y = y * SimToSI.angular_factor
    ambf_obj.set_rpy(r, p, y)


def get_joint_factor(joint_type):
    if joint_type == JointType.PRISMATIC:
        factor = SimToSI.linear_factor
    elif joint_type == JointType.REVOLUTE:
        factor = SimToSI.angular_factor
    else:
        raise 'ERROR! JOINT TYPE INVALID'
    return factor


def get_joint_pos(ambf_obj, idx, joint_type):
    factor = get_joint_factor(joint_type)
    return ambf_obj.get_joint_pos(idx) / factor


def set_joint_pos(ambf_obj, idx, joint_type, cmd):
    factor = get_joint_factor(joint_type)
    return ambf_obj.set_joint_pos(idx, cmd * factor)


def get_joint_vel(ambf_obj, idx, joint_type):
    factor = get_joint_factor(joint_type)
    return ambf_obj.get_joint_vel(idx) / factor


def set_joint_vel(ambf_obj, idx, joint_type, cmd):
    factor = get_joint_factor(joint_type)
    return ambf_obj.set_joint_vel(idx, cmd * factor)
