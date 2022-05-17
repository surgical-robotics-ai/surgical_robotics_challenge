import time

from surgical_robotics_challenge.utils.utilities import *
from ambf_msgs.msg import RigidBodyState
from PyKDL import Frame, Rotation, Vector
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
from collections import deque


def frame_to_pose_stamped_msg(frame):
    """

    :param frame:
    :return:
    """
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = frame.p[0]
    msg.pose.position.y = frame.p[1]
    msg.pose.position.z = frame.p[2]

    msg.pose.orientation.x = frame.M.GetQuaternion()[0]
    msg.pose.orientation.y = frame.M.GetQuaternion()[1]
    msg.pose.orientation.z = frame.M.GetQuaternion()[2]
    msg.pose.orientation.w = frame.M.GetQuaternion()[3]

    return msg


def pose_stamped_msg_to_frame(msg):
    """

    :param msg:
    :return:
    """
    return pose_msg_to_frame(msg.pose)


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


def ambf_obj_pose_to_frame(obj):
    """

    :param obj:
    :return:
    """
    p = Vector(obj.get_pos().x,
               obj.get_pos().y,
               obj.get_pos().z)
    R = Rotation.Quaternion(obj.get_rot().x,
                            obj.get_rot().y,
                            obj.get_rot().z,
                            obj.get_rot().w)
    return Frame(R, p)


class NeedleKinematics:
    def __init__(self):
        """

        :return:
        """
        self._needle_sub = rospy.Subscriber('/ambf/env/Needle/State', RigidBodyState, self.needle_cb, queue_size=1)
        # Base in Needle Origin
        self._T_bINn = Frame(Rotation.RPY(0., 0., 0.), Vector(-0.102, 0., 0.))
        # Mid in Needle Origin
        self._T_mINn = Frame(Rotation.RPY(0., 0., -1.091), Vector(-0.048, 0.093, 0.))
        # Tip in Needle Origin
        self._T_tINn = Frame(Rotation.RPY(0., 0., -0.585), Vector(0.056, 0.085, 0.))
        # Needle in World
        self._T_nINw = Frame()

    def needle_cb(self, msg):
        """

        :param msg:
        :return:
        """
        self._T_nINw = pose_msg_to_frame(msg.pose)

    def get_tip_pose(self):
        """

        :return:
        """
        T_tINw = self._T_nINw * self._T_tINn
        return T_tINw

    def get_base_pose(self):
        """

        :return:
        """
        T_bINw = self._T_nINw * self._T_bINn
        return T_bINw

    def get_mid_pose(self):
        """

        :return:
        """
        T_mINw = self._T_nINw * self._T_mINn
        return T_mINw

    def get_pose(self):
        return self._T_nINw


class Task_1_Evaluation:
    def __init__(self, client, team_name):
        """

        :param client:
        :param team_name:
        :return:
        """
        self._world = client.get_world_handle()
        self._needle_kinematics = NeedleKinematics()
        self._ecm_sub = rospy.Subscriber('/ambf/env/CameraFrame/State', RigidBodyState, self._ecm_cb, queue_size=1)
        self._T_ecmINw = Frame()
        self._team_name = team_name
        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + self._team_name
        self._task_sub = rospy.Subscriber(prefix + '/task1/', PoseStamped, self.task_completion_cb, queue_size=1)

        time.sleep(1.0)
        self._start_time = self._world._state.sim_time
        self._completion_time = self._start_time + 60.0
        self._T_nINw_reported = Frame()
        self._done = False

    def _ecm_cb(self, msg):
        """

        :param msg:
        :return:
        """
        self._T_ecmINw = pose_msg_to_frame(msg.pose)

    def task_completion_cb(self, msg):
        """

        :param msg:
        :return:
        """
        self._completion_time = self._world._state.sim_time - self._start_time
        T_nINe = pose_stamped_msg_to_frame(msg)
        self._T_nINw_reported = self._T_ecmINw * T_nINe
        self._done = True

    def print_evaluation(self, T_reported, T_actual):
        """

        :param T_reported:
        :param T_actual:
        :return:
        """
        print('Team: ', self._team_name, ' Task 1 Completion Report: ')

        P_base_reported = (T_reported * self._needle_kinematics.get_base_pose()).p
        P_base_actual = (T_actual * self._needle_kinematics.get_base_pose()).p
        e_base = (P_base_actual - P_base_reported).Norm()
        print('\t Base Error: ', e_base)

        P_mid_reported = (T_reported * self._needle_kinematics.get_mid_pose()).p
        P_mid_actual = (T_actual * self._needle_kinematics.get_mid_pose()).p
        e_mid = (P_mid_actual - P_mid_reported).Norm()
        print('\t Mid Error: ', e_mid)

        P_tip_reported = (T_reported * self._needle_kinematics.get_tip_pose()).p
        P_tip_actual = (T_actual * self._needle_kinematics.get_tip_pose()).p
        e_tip = (P_tip_actual - P_tip_reported).Norm()
        print('\t Tip Error: ', e_tip)

        print('\t Completion Time: ', self._completion_time)
        print('\t Task 1 Overall Score (Lower is Better): ', e_tip + e_mid + e_base)

    def evaluate(self):
        """

        :return:
        """
        while not self._done:
            time.sleep(1.0)
            print('[', time.time(), '] Waiting for task 1 completion report')

        T_nINw_actual = self._needle_kinematics.get_pose()
        self.print_evaluation(self._T_nINw_reported, T_nINw_actual)


class SceneKinematics:
    def __init__(self, hole_count):
        """

        :param hole_count:
        :return:
        """
        self._hole_count = hole_count
        self.T_entriesINw = [Frame()] * self._hole_count
        self.T_exitsINw = [Frame()] * self._hole_count
        self.T_ntINw = Frame()
        self.t = 0.0


class Task_2_Evaluation:
    def __init__(self, client, team_name):
        """

        :param client:
        :param team_name:
        :return:
        """
        self._world = client.get_world_handle()
        self._needle_kinematics = NeedleKinematics()
        self._team_name = team_name
        self._hole_count = 4
        self._entry_points = []
        self._exit_points = []
        for i in range(self._hole_count):
            self._entry_points.append(client.get_obj_handle("Entry" + str(i+1)))
            self._exit_points.append(client.get_obj_handle("Exit" + str(i+1)))

        self._scene_trajectories = deque()

        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + self._team_name
        self._task_sub = rospy.Subscriber(prefix + '/task2/', Bool, self.task_completion_cb, queue_size=1)

        self._start_time = self._world._state.sim_time
        self._completion_time = self._start_time + 60.0
        self._done = False
        # Needle protruding from the exit as the end of task
        self._L_ntINexit_axial = 0.0
        # Cross-sectional distance from the exit hole's center
        self._L_ntINexit_lateral = 0.0
        # Cross-sectional distance from the entry hole's center
        self._L_ntINentry_lateral = 0.0

        self._entry_exit_idx = -1

    def update_scene_trajectories(self):
        """

        :return:
        """
        SK = SceneKinematics(self._hole_count)
        for i in range(self._hole_count):
            SK.T_entriesINw[i] = ambf_obj_pose_to_frame(self._entry_points[i])
            SK.T_exitsINw[i] = ambf_obj_pose_to_frame(self._exit_points[i])
        SK.T_ntINw = self._needle_kinematics.get_tip_pose()
        SK.t = self._world._state.sim_time
        self._scene_trajectories.append(SK)

    def task_completion_cb(self, msg):
        """

        :param msg:
        :return:
        """
        if msg.data is False:
            print('Error!, Task 2 Completion Message must be True')

        self._completion_time = self._world._state.sim_time - self._start_time
        self._done = True

    def print_evaluation(self):
        """

        :return:
        """
        print('Team: ', self._team_name, ' Task 2 Completion Report: ')
        print('\t Completion Time: ', self._completion_time)
        print('\t Entry/Exit Targeted Hole: ', self._entry_exit_idx + 1)
        print('\t Needle Tip Axial Distance From Exit Hole (Lower is Better): ', self._L_ntINexit_axial)
        print('\t Needle Tip Lateral Distance From Exit Hole (Lower is Better): ', self._L_ntINexit_lateral)
        print('\t Needle Tip Lateral Distance From Entry Hole (Lower is Better): ', self._L_ntINentry_lateral)

    def evaluate(self):
        """

        :return:
        """
        t = 0.0
        while not self._done:
            time.sleep(0.01)
            self.update_scene_trajectories()
            t = t + 0.01
            if t % 1.0 >= 0.99:
                print(time.time(), ' ) Waiting for task 2 completion report')

        # Record the final trajectories
        self.update_scene_trajectories()
        print('Completion Report Submitted, Running evaluation')
        print(len(self._scene_trajectories))
        # Find the closes exit to the needle tip
        Traj_last = self._scene_trajectories[-1]
        closest_exit_idx = self.find_closest_exit_to_needle_tip(Traj_last)
        T_exitINw = Traj_last.T_exitsINw[closest_exit_idx]
        T_ntINw = Traj_last.T_ntINw

        self._L_ntINexit_axial = self.compute_needle_axial_distance_from_hole(T_exitINw, T_ntINw)

        self._L_ntINexit_lateral, traj_idx = self.compute_needle_lateral_distance_from_exit(closest_exit_idx, traj_offset=1)

        # Compute a new offset to start the search
        traj_offset = len(self._scene_trajectories) - traj_idx
        closest_entry_idx = closest_exit_idx
        self._L_ntINentry_lateral, traj_idx = self.compute_needle_lateral_distance_from_entry(closest_entry_idx, traj_offset)
        self._entry_exit_idx = closest_entry_idx
        self.print_evaluation()

    def find_closest_exit_to_needle_tip(self, Traj):
        """

        :param Traj:
        :return:
        """
        E_ntINexits = [Frame()] * Traj._hole_count
        for i in range(Traj._hole_count):
            E_ntINexits[i] = (Traj.T_exitsINw[i].Inverse() * Traj.T_ntINw).p.Norm()

        # Find closes entry
        min_idx = np.argmin(E_ntINexits)
        return min_idx

    def compute_needle_axial_distance_from_hole(self, T_hINw, T_ntINw):
        """

        :param T_hINw:
        :param T_ntINw:
        :return:
        """
        P_ntINhole = (T_hINw.Inverse() * T_ntINw).p
        # The z value of the above vector is the axial distance between the needle tip
        # and the hole origin
        return P_ntINhole[2]

    def compute_needle_lateral_distance_from_hole(self, T_hINw, T_ntINw):
        """

        :param T_hINw:
        :param T_ntINw:
        :return:
        """
        P_ntINhole = (T_hINw.Inverse() * T_ntINw).p
        P_ntINhole[2] = 0.
        return P_ntINhole.Norm()

    def compute_needle_lateral_distance_from_exit(self, exit_idx, traj_offset):
        """

        :param exit_idx:
        :param traj_offset:
        :return:
        """
        min_traj_idx = -1
        # Check trajectory to find where the needle tip passed through the exit hole
        L_min = 0.1
        L_max = -0.1
        end_traj_idx = (len(self._scene_trajectories) - 1) - traj_offset
        for i in range(end_traj_idx, -1, -1):
            Traj = self._scene_trajectories[i]
            L = self.compute_needle_axial_distance_from_hole(Traj.T_exitsINw[exit_idx], Traj.T_ntINw)
            print(i, ') L: ', L, ' | Exit Idx: ', exit_idx)
            if abs(L) < L_min:
                L_min = L
                min_traj_idx = i
            if min_traj_idx != -1 and L < L_max: # Needle is too far in, break search
                break

        if min_traj_idx == -1:
            raise Exception('Error, unable to compute lateral distance between needle tip and exit hole')

        T_exitINw = self._scene_trajectories[min_traj_idx].T_exitsINw[exit_idx]
        T_ntINw = self._scene_trajectories[min_traj_idx].T_ntINw
        lateral_distance = self.compute_needle_lateral_distance_from_hole(T_exitINw, T_ntINw)
        return lateral_distance, min_traj_idx

    def compute_needle_lateral_distance_from_entry(self, entry_idx, traj_offset):
        """

        :param entry_idx:
        :param traj_offset:
        :return:
        """
        min_traj_idx = -1
        # Check trajectory to find where the needle tip passed through the entry hole
        L_min = 0.1
        L_max = 0.1
        end_traj_idx = (len(self._scene_trajectories) - 1) - traj_offset
        for i in range(end_traj_idx, -1, -1):
            Traj = self._scene_trajectories[i]
            L = self.compute_needle_axial_distance_from_hole(Traj.T_entriesINw[entry_idx], Traj.T_ntINw)
            if abs(L) < L_min:
                L_min = L
                min_traj_idx = i
            if min_traj_idx != -1 and L > L_max:  # Needle is too far in, break search
                break

        if min_traj_idx == -1:
            raise 'Error, unable to compute lateral distance between needle tip and entry hole'

        T_entryINw = self._scene_trajectories[min_traj_idx].T_entriesINw[entry_idx]
        T_ntINw = self._scene_trajectories[min_traj_idx].T_ntINw
        lateral_distance = self.compute_needle_lateral_distance_from_hole(T_entryINw, T_ntINw)
        return lateral_distance, min_traj_idx
