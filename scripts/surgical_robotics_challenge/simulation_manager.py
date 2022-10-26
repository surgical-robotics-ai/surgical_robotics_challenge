from ambf_client import Client
import surgical_robotics_challenge.units_conversion as units_conversion


class SimulationObject:
    def __init__(self, ambf_object):
        self._object = ambf_object
        self._joint_type = None # To distinguish between revolute and prismatic joints

    def set_joint_types(self, joint_types):
        self._joint_type = joint_types

    def get_pos(self):
        return units_conversion.get_pos(self._object)

    def get_rotation(self):
        return units_conversion.get_rotation(self._object)

    def get_pose(self):
        return units_conversion.get_pose(self._object)

    def set_pos(self, pos):
        units_conversion.set_pos(self._object, pos)

    def set_pose(self, pose):
        units_conversion.set_pos(self._object, pose.p)
        self.set_rotation(pose.M)

    def set_rpy(self, r, p, y):
        units_conversion.set_rpy(self._object, r, p, y)

    def set_rotation(self, R):
        rpy = R.GetRPY()
        units_conversion.set_rpy(self._object, rpy[0], rpy[1], rpy[2])

    def get_joint_pos(self, idx):
        return units_conversion.get_joint_pos(self._object, idx, self._joint_type[idx])

    def set_joint_pos(self, idx, cmd):
        units_conversion.set_joint_pos(self._object, idx, self._joint_type[idx], cmd)

    def get_joint_vel(self, idx):
        return units_conversion.get_joint_vel(self._object, idx, self._joint_type[idx])

    def set_joint_vel(self, idx, cmd):
        units_conversion.set_joint_vel(self._object, idx, self._joint_type[idx], cmd)

    def get_joint_names(self):
        return self._object.get_joint_names()

    def set_force(self, f):
        self._object.set_force(f[0], f[0], f[0])

    def set_torque(self, t):
        self._object.set_torque(t[0], t[0], t[0])


class SimulationManager:
    def __init__(self, name):
        self._client = Client(name)
        self._client.connect()

    def get_obj_handle(self, name):
        ambf_object = self._client.get_obj_handle(name)
        if ambf_object:
            return SimulationObject(ambf_object)
        else:
            return None

    def get_world_handle(self):
        return self._client.get_world_handle()
