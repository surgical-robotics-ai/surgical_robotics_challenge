from PyKDL import Frame, Rotation, Vector
import numpy as np
import rospy
from ambf_msgs.msg import RigidBodyState

def pose_msg_to_frame(msg):
    """

    :param msg:
    :return:
    """
    p = Vector(msg.position.x,
               msg.position.y,
               msg.position.z)

    R = Rotation.Quaternion(msg.orientation.x,
                            msg.orientation.y,
                            msg.orientation.z,
                            msg.orientation.w)

    return Frame(R, p)

class NeedleKinematics:
    # Base in Needle Origin
    T_bINn = Frame(Rotation.RPY(0., 0., 0.), Vector(-0.102, 0., 0.))
    # Mid in Needle Origin
    T_mINn = Frame(Rotation.RPY(0., 0., -1.091), Vector(-0.048, 0.093, 0.))
    # Tip in Needle Origin
    T_tINn = Frame(Rotation.RPY(0., 0., -0.585), Vector(0.056, 0.085, 0.))

    def __init__(self):
        """

        :return:
        """
        self._needle_sub = rospy.Subscriber(
            '/ambf/env/Needle/State', RigidBodyState, self.needle_cb, queue_size=1)
        # Needle in World
        self._T_nINw = Frame()

    def needle_cb(self, msg):
        """ needle callback; called every time new msg is received

        :param msg:
        :return:
        """
        self._T_nINw = pose_msg_to_frame(msg.pose)

    def get_tip_pose(self):
        """

        :return:
        """
        T_tINw = self._T_nINw * self.T_tINn
        return T_tINw

    def get_base_pose(self):
        """

        :return:
        """
        T_bINw = self._T_nINw * self.T_bINn
        return T_bINw

    def get_mid_pose(self):
        """

        :return:
        """
        T_mINw = self._T_nINw * self.T_mINn
        return T_mINw

    def get_pose(self):
        return self._T_nINw
