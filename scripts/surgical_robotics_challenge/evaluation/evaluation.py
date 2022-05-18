import time

from surgical_robotics_challenge.utils.utilities import *
from ambf_msgs.msg import RigidBodyState
from PyKDL import Frame, Rotation, Vector
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
from collections import deque
from enum import Enum


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


class Task_2_Evaluation_Report():
    def __init__(self):
        self._team_name = None
        # Needle protruding from the exit as the end of task
        self._L_ntINexit_axial = 0.0
        # Cross-sectional distance from the exit hole's center
        self._L_ntINexit_lateral = 0.0
        # Cross-sectional distance from the entry hole's center
        self._L_ntINentry_lateral = 0.0

        self._entry_exit_idx = -1

        self._completion_time = 100000.0

    def print(self):
        """

        :return:
        """
        print('Team: ', self._team_name, ' Task 2 Completion Report: ')
        print('\t Completion Time: ', self._completion_time)
        print('\t Entry/Exit Targeted Hole: ', self._entry_exit_idx + 1)
        print('\t Needle Tip Axial Distance From Exit Hole (Lower is Better): ', self._L_ntINexit_axial)
        print('\t Needle Tip Lateral Distance From Exit Hole (Lower is Better): ', self._L_ntINexit_lateral)
        print('\t Needle Tip Lateral Distance From Entry Hole (Lower is Better): ', self._L_ntINentry_lateral)


class HoleType(Enum):
    ENTRY = 0
    EXIT = 1


class SceneKinematicsFrame:
    def __init__(self, hole_count):
        """

        :param hole_count:
        :return:
        """
        self._hole_count = hole_count
        self.T_holesINw = dict()
        self.T_holesINw[HoleType.ENTRY] = [Frame()] * self._hole_count
        self.T_holesINw[HoleType.EXIT] = [Frame()] * self._hole_count
        self.T_ntINw = Frame()
        self.t = 0.0


class NeedleContactEvent:
    def __init__(self):
        self._hole_type = None
        self._T_ntINhole = Frame()
        self._t = 0.0
        self._hole_idx = -1
        # The Object Aligned Bounding Box to check for needle tip
        self._hole_bounds = Vector(0.05, 0.05, 0.05)

    def compute_needle_hole_collision(self, T_ntINhole):
        for j in range(3):
            if abs(T_ntINhole.p[j]) > self._hole_bounds[j]:
                # Needle tip is out of bounds, ignore
                return True
        return False


class Task_2_Evaluation():
    def __init__(self, client, team_name):
        """

        :param client:
        :param team_name:
        :return:
        """
        self._world = client.get_world_handle()
        self._needle_kinematics = NeedleKinematics()
        self._hole_count = 4
        self._hole_objs = dict()
        self._hole_objs[HoleType.ENTRY] = []
        self._hole_objs[HoleType.EXIT] = []
        for i in range(self._hole_count):
            self._hole_objs[HoleType.ENTRY].append(client.get_obj_handle("Entry" + str(i+1)))
            self._hole_objs[HoleType.EXIT].append(client.get_obj_handle("Exit" + str(i+1)))

        self._scene_trajectories = deque()
        self._needle_holes_proximity_events = dict()
        self._needle_holes_proximity_events[HoleType.ENTRY] = [deque()]*self._hole_count
        self._needle_holes_proximity_events[HoleType.EXIT] = [deque()]*self._hole_count

        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + team_name
        self._task_sub = rospy.Subscriber(prefix + '/task2/', Bool, self.task_completion_cb, queue_size=1)

        self._done = False
        self._report = Task_2_Evaluation_Report()
        self._report._team_name = team_name
        self._entry_exit_idx = -1
        self._start_time = self._world._state.sim_time

    def capture_scene_kinematics(self):
        """

        :return:
        """
        SKF = SceneKinematicsFrame(self._hole_count)
        SKF.T_ntINw = self._needle_kinematics.get_tip_pose()

        for hole_type in HoleType:
            for i in range(self._hole_count):
                SKF.T_holesINw[hole_type][i] = ambf_obj_pose_to_frame(self._hole_objs[hole_type][i])

                # Check for needle tip contact with entry
                T_ntINhole = SKF.T_holesINw[hole_type][i].Inverse() * SKF.T_ntINw
                self.compute_needle_hole_proximity_event(T_ntINhole, hole_type, i, self._world._state.sim_time)

        self._scene_trajectories.append(SKF)

    def compute_needle_hole_proximity_event(self, T_ntINhole, hole_type, hole_idx, time):
        ne = NeedleContactEvent()
        if not ne.compute_needle_hole_collision(T_ntINhole):
            ne._hole_type = hole_type
            ne._T_ntINhole = T_ntINhole
            ne._t = time
            ne._hole_idx = hole_idx
            self._needle_holes_proximity_events[hole_type][hole_idx].append(ne)
            print('\t\t', ne._hole_type, ne._hole_idx, ne._T_ntINhole.p.Norm())

    def task_completion_cb(self, msg):
        """

        :param msg:
        :return:
        """
        if msg.data is False:
            print('Error!, Task 2 Completion Message must be True')

        self._completion_time = self._world._state.sim_time - self._start_time
        self._done = True

    def evaluate(self):
        """

        :return:
        """
        t = 0.0
        while not self._done:
            time.sleep(0.01)
            self.capture_scene_kinematics()
            t = t + 0.01
            if t % 1.0 >= 0.99:
                print(time.time(), ' ) Waiting for task 2 completion report')

        # Record the final trajectories
        self.capture_scene_kinematics()
        print('Completion Report Submitted, Running evaluation')
        print(len(self._needle_hole_proximity_events))

        self._report.print()
