from PyKDL import Frame, Rotation, Vector, Twist
import time


def ambf_pose_to_frame(obj):
    p = Vector(obj.get_pos().x, obj.get_pos().y, obj.get_pos().z)
    R = Rotation.Quaternion(obj.get_rot().x, obj.get_rot().y, obj.get_rot().z, obj.get_rot().w)
    T = Frame(R, p)
    return T


class Scene:
    def __init__(self, client):
        self.client = client
        self._needle = self.client.get_obj_handle("Needle")
        self._entry1 = self.client.get_obj_handle("Entry1")
        self._entry2 = self.client.get_obj_handle("Entry2")
        self._entry3 = self.client.get_obj_handle("Entry3")
        self._entry4 = self.client.get_obj_handle("Entry4")
        self._exit1 = self.client.get_obj_handle("Exit1")
        self._exit2 = self.client.get_obj_handle("Exit2")
        self._exit3 = self.client.get_obj_handle("Exit3")
        self._exit4 = self.client.get_obj_handle("Exit4")
        time.sleep(0.1)

    def needle_measured_cp(self):
        return ambf_pose_to_frame(self._needle)

    def entry1_measured_cp(self):
        return ambf_pose_to_frame(self._entry1)

    def entry2_measured_cp(self):
        return ambf_pose_to_frame(self._entry2)

    def entry3_measured_cp(self):
        return ambf_pose_to_frame(self._entry3)

    def entry4_measured_cp(self):
        return ambf_pose_to_frame(self._entry4)

    def exit1_measured_cp(self):
        return ambf_pose_to_frame(self._exit1)

    def exit2_measured_cp(self):
        return ambf_pose_to_frame(self._exit2)

    def exit3_measured_cp(self):
        return ambf_pose_to_frame(self._exit3)

    def exit4_measured_cp(self):
        return ambf_pose_to_frame(self._exit4)
