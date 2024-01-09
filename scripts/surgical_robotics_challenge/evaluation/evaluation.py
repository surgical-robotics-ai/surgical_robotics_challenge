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
from ambf_client import Client
from argparse import ArgumentParser
from surgical_robotics_challenge import units_conversion


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


class GlobalParams:
    hole_count = 4
    # The Object Aligned Bounding Box (OABB) to check for needle tip
    hole_bounds = Vector(0.005, 0.005, 0.005) # in SI
    insertion_depth_threshold = 0.001 # in SI


class NeedleKinematics:
    # Base in Needle Origin
    T_bINn = Frame(Rotation.RPY(0., 0., 0.), Vector(-0.102, 0., 0.) / units_conversion.SimToSI.linear_factor)
    # Mid in Needle Origin
    T_mINn = Frame(Rotation.RPY(0., 0., -1.091), Vector(-0.048, 0.093, 0.) / units_conversion.SimToSI.linear_factor)
    # Tip in Needle Origin
    T_tINn = Frame(Rotation.RPY(0., 0., -0.585), Vector(0.056, 0.085, 0.) / units_conversion.SimToSI.linear_factor)

    def __init__(self):
        """

        :return:
        """
        self._needle_sub = rospy.Subscriber('/ambf/env/Needle/State', RigidBodyState, self.needle_cb, queue_size=1)
        # Needle in World
        self._T_nINw = Frame()

    def needle_cb(self, msg):
        """

        :param msg:
        :return:
        """
        T_nINw = pose_msg_to_frame(msg.pose)
        T_nINw.p = T_nINw.p / units_conversion.SimToSI.linear_factor
        self._T_nINw = T_nINw

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


class Task_1_Evaluation_Report():
    def __init__(self):
        """

        """
        self.team_name = None

        self.E_base = None

        self.E_mid = None

        self.E_tip = None

        self.completion_time = None

        self.success = False

    def print_report(self):
        """

        :return:
        """

        print('Team: ', self.team_name, ' Task 1 Completion Report: ')
        if self.success:
            print(OK_STR('\t Task Successful: '))
            print('\t Completion Time: ', self.completion_time)
            print('\t Base Error: ', self.E_base)
            print('\t Mid Error: ', self.E_mid)
            print('\t Tip Error: ', self.E_tip)
            print('\t Task 1 Overall Score (Lower is Better): ', self.E_base + self.E_mid + self.E_tip)
        else:
            print(FAIL_STR('Task Failed: '))


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
        self._start_time = rospy.Time.now().to_sec()
        self._T_nINw_reported = Frame()
        self._done = False
        self._report = Task_1_Evaluation_Report()

    def _ecm_cb(self, msg):
        """

        :param msg:
        :return:
        """
        T_ecmINw = pose_msg_to_frame(msg.pose)
        T_ecmINw.p = T_ecmINw.p / units_conversion.SimToSI.linear_factor
        self._T_ecmINw = T_ecmINw

    def task_completion_cb(self, msg):
        """

        :param msg:
        :return:
        """
        self._report.completion_time = rospy.Time.now().to_sec() - self._start_time
        T_nINe = pose_stamped_msg_to_frame(msg)
        self._T_nINw_reported = self._T_ecmINw * T_nINe
        self._done = True

    def evaluate(self):
        """

        :return:
        """
        while not self._done:
            time.sleep(1.0)
            print('[', time.time(), '] Waiting for task completion report')

        T_nINw = self._needle_kinematics.get_pose()

        P_b_r = (self._T_nINw_reported * NeedleKinematics.T_bINn).p
        P_m_r = (self._T_nINw_reported * NeedleKinematics.T_mINn).p
        P_t_r = (self._T_nINw_reported * NeedleKinematics.T_tINn).p

        P_b = (T_nINw * NeedleKinematics.T_bINn).p
        P_m = (T_nINw * NeedleKinematics.T_mINn).p
        P_t = (T_nINw * NeedleKinematics.T_tINn).p

        self._report.success = True
        self._report.E_base = (P_b - P_b_r).Norm()
        self._report.E_mid = (P_m - P_m_r).Norm()
        self._report.E_tip = (P_t - P_t_r).Norm()
        self._report.print_report()


class Task_2_Evaluation_Report():
    def __init__(self):
        """

        """
        self.team_name = None

        # Needle protruding from the exit as the end of task
        self.L_ntINexit_axial = None

        # Cross-sectional distance from the exit hole's center
        self.L_ntINexit_lateral = None

        # Cross-sectional distance from the entry hole's center
        self.L_ntINentry_lateral = None

        # Cross-sectional distance from the exit hole's center
        self.P_max_ntINexit_lateral = None

        # Cross-sectional distance from the entry hole's center
        self.P_max_ntINentry_lateral = None

        # Cross-sectional distance from the exit hole's center
        self.P_ntINexit_lateral = None

        # Cross-sectional distance from the entry hole's center
        self.P_ntINentry_lateral = None

        self.entry_exit_idx = None

        self.completion_time = None

        self.success = False

    def print_report(self):
        """

        :return:
        """
        print('Team: ', self.team_name, ' Task 2 Completion Report: ')
        if self.success:
            print(OK_STR('\t Task Successful: '))
            print('\t Completion Time: ', self.completion_time)
            print('\t Targeted Entry/Exit Hole Pair (1 to 4): ', self.entry_exit_idx + 1)
            print('\t Needle Tip Axial Distance From Exit Hole (Recommended {})'.format(GlobalParams.hole_bounds[0]), self.L_ntINexit_axial)
            print('\t Needle Tip P From Entry Hole During Insertion (Lower is Better): ',
                  self.P_ntINentry_lateral)
            print('\t Needle Tip P Exit Hole During Insertion (Lower is Better): ',
                  self.P_ntINexit_lateral)
            print('\t Needle Tip Lateral Distance From Entry Hole During Insertion (Lower is Better): ',
                  self.L_ntINentry_lateral)
            print('\t Needle Tip Lateral Distance From Exit Hole During Insertion (Lower is Better): ',
                  self.L_ntINexit_lateral)
            print('\t Needle Tip Max Lateral Component From Entry Hole During Insertion (Lower is Better): ',
                  self.P_max_ntINentry_lateral)
            print('\t Needle Tip Max Lateral Component From Exit Hole During Insertion (Lower is Better): ',
                  self.P_max_ntINexit_lateral)
        else:
            print(FAIL_STR('Task Failed: '))


class HoleType(Enum):
    """

    """
    ENTRY = 0
    EXIT = 1


class SceneKinematicsFrame:
    def __init__(self):
        """

        :param hole_count:
        :return:
        """
        self.T_holesINw = dict()
        self.T_holesINw[HoleType.ENTRY] = [Frame() for _ in range(GlobalParams.hole_count)]
        self.T_holesINw[HoleType.EXIT] = [Frame() for _ in range(GlobalParams.hole_count)]
        self.T_ntINw = Frame()
        self.t = 0.0

    def find_closest_hole_to_needle_tip(self):
        NCE = NeedleContactEvent()
        min_distance = 10000.
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                T = self.T_holesINw[hole_type][hidx].Inverse() * self.T_ntINw
                if T.p.Norm() < min_distance:
                    min_distance = T.p.Norm()
                    NCE.T_ntINhole = T
                    NCE.hole_type = hole_type
                    NCE.hole_idx = hidx
                    NCE.t = self.t
        return NCE


class NeedleContactEvent:
    def __init__(self):
        """

        """
        self.hole_type = None
        self.T_ntINhole = Frame()
        self.t = 0.0
        self.hole_idx = -1

    def is_needle_intersecting_with_hole(self, T_ntINhole):
        """

        :param T_ntINhole:
        :return:
        """
        for j in range(3):
            if abs(T_ntINhole.p[j]) > GlobalParams.hole_bounds[j]:
                # Needle tip is out of bounds, ignore
                return False
        return True


class ContactEventHelper:
    @staticmethod
    def validate_needle_event(hole_type, hole_idx, NE, print_output=True):
        if NE.hole_type != hole_type or NE.hole_idx != hole_idx:
            if print_output:
                print('ERROR! For hole_type: ', hole_type, ' and hole_idx: ', hole_idx,
                      ' NE hole_type: ', NE.hole_type, ' hole_idx: ', NE.hole_idx)
            return False
        else:
            return True

    @staticmethod
    def validate_needle_insertion_events(needle_holes_proximity_events):
        incorrect_events = 0
        total_events = 0
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                event_count = len(needle_holes_proximity_events[hole_type][hidx])
                for e in range(event_count):
                    NE = needle_holes_proximity_events[hole_type][hidx][e]
                    total_events = total_events + 1
                    if not ContactEventHelper.validate_needle_event(hole_type, hidx, NE, print_output=False):
                        incorrect_events = incorrect_events + 1
        print('Total Events: ', total_events, ' Incorrect Events: ', incorrect_events)

    @staticmethod
    def compute_insertion_events_from_proximity_events(needle_holes_proximity_events):
        hole_insertion_events = []
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                event_size = len(needle_holes_proximity_events[hole_type][hidx])
                i_insertion = -1
                if event_size < 2:
                    # No insertion to report as we only have a single point within hole's OABB
                    pass
                else:
                    for ev in range(event_size-1, 0, -1):
                        z1 = needle_holes_proximity_events[hole_type][hidx][ev].T_ntINhole.p[2]
                        z0 = needle_holes_proximity_events[hole_type][hidx][ev-1].T_ntINhole.p[2]

                        if hole_type is HoleType.ENTRY:
                            if z1 < 0. < z0:
                                i_insertion = ev
                                break
                        elif hole_type is HoleType.EXIT:
                            if z1 > 0. > z0:
                                i_insertion = ev
                                break
                        else:
                            raise Exception('Cannot Happen')

                    # For debugging
                    if i_insertion == -1:
                        depths = []
                        for ev in range(event_size):
                            z = needle_holes_proximity_events[hole_type][hidx][ev].T_ntINhole.p[2]
                            depths.append(z)
                        print('Error! For hole_type', hole_type, 'hole_idx', hidx)
                        print('Error! Not able to find insertion from depths')
                        print(depths)

                if i_insertion != -1:
                    NCE = needle_holes_proximity_events[hole_type][hidx][i_insertion]
                    # ContactEventHelper.validate_needle_event(hole_type, hidx, NCE, print_output=True)
                    hole_insertion_events.append(NCE)
        # Sort the list based on time events
        hole_insertion_events.sort(key=lambda x: x.t, reverse=True)
        return hole_insertion_events

    @staticmethod
    def compute_axial_distance_from_hole(T_ntINhole):
        """

        :return:
        """
        # The axial distance is along the z axes
        return abs(T_ntINhole.p[2])

    @staticmethod
    def compute_lateral_distance_from_hole(T_ntINhole):
        """

        :return:
        """
        p = T_ntINhole.p
        p[2] = 0.
        return p.Norm()

    @staticmethod
    def compute_max_lateral_component_from_hole(T_ntINhole):
        """

        :return:
        """
        p = T_ntINhole.p
        px = abs(p[0])
        py = abs(p[1])
        return max([px, py])


class Task_2_Evaluation():
    def __init__(self, client, team_name):
        """

        :param client:
        :param team_name:
        :return:
        """
        self._world = client.get_world_handle()
        self._needle_kinematics = NeedleKinematics()
        self._hole_objs = dict()
        self._hole_objs[HoleType.ENTRY] = []
        self._hole_objs[HoleType.EXIT] = []
        for i in range(GlobalParams.hole_count):
            self._hole_objs[HoleType.ENTRY].append(client.get_obj_handle("Entry" + str(i+1)))
            self._hole_objs[HoleType.EXIT].append(client.get_obj_handle("Exit" + str(i+1)))

        self._scene_trajectories = deque()
        self._needle_holes_proximity_events = dict()
        self._needle_holes_proximity_events[HoleType.ENTRY] = [deque() for _ in range(GlobalParams.hole_count)]
        self._needle_holes_proximity_events[HoleType.EXIT] = [deque() for _ in range(GlobalParams.hole_count)]

        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + team_name
        self._task_sub = rospy.Subscriber(prefix + '/task2/', Bool, self.task_completion_cb, queue_size=1)

        self._done = False
        self._report = Task_2_Evaluation_Report()
        self._report.team_name = team_name
        self._entry_exit_idx = -1
        self._start_time = rospy.Time.now().to_sec()
        time.sleep(1.0)

    def capture_scene_kinematics(self):
        """

        :return:
        """
        SKF = SceneKinematicsFrame()
        SKF.t = self._world._state.sim_time
        SKF.T_ntINw = self._needle_kinematics.get_tip_pose()

        for hole_type in HoleType:
            for i in range(GlobalParams.hole_count):
                SKF.T_holesINw[hole_type][i] = units_conversion.get_pose(self._hole_objs[hole_type][i])

        self._scene_trajectories.append(SKF)
        return SKF

    def compute_needle_hole_proximity_event(self, SKF):
        """

        :param SKF:
        :return:
        """
        proximity_events = []
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                T_ntINhole = SKF.T_holesINw[hole_type][hidx].Inverse() * SKF.T_ntINw
                ne = NeedleContactEvent()
                if ne.is_needle_intersecting_with_hole(T_ntINhole):
                    ne.hole_type = hole_type
                    ne.hole_idx = hidx
                    ne.T_ntINhole = T_ntINhole
                    ne.t = SKF.t
                    # ContactEventHelper.validate_needle_event(hole_type, hidx, ne)
                    (self._needle_holes_proximity_events[hole_type][hidx]).append(ne)
                    proximity_events.append(ne)
                    # print('\t\t', ne.hole_type, ne.hole_idx, ne.T_ntINhole.p.Norm())
        return proximity_events

    def task_completion_cb(self, msg):
        """

        :param msg:
        :return:
        """
        if msg.data is False:
            print('Error!, Task 2 Completion Message must be True')

        self._report.completion_time = rospy.Time.now().to_sec() - self._start_time
        self._done = True

    def evaluate(self):
        """

        :return:
        """
        t = 0.0
        while not self._done:
            time.sleep(0.01)
            SKF = self.capture_scene_kinematics()
            self.compute_needle_hole_proximity_event(SKF)
            t = t + 0.01
            if t % 1.0 >= 0.99:
                print(time.time(), ' ) Waiting for task completion report')

        # Record the final trajectories
        SKF = self.capture_scene_kinematics()
        print('Completion Report Submitted, Running evaluation')

        NCE = SKF.find_closest_hole_to_needle_tip()

        # self.validate_needle_insertion_events()

        self._report.success = False # Initialize to false
        if NCE.hole_type is HoleType.EXIT:
            insertion_events = ContactEventHelper.compute_insertion_events_from_proximity_events(
                self._needle_holes_proximity_events)
            if len(insertion_events) < 2:
                # Failed
                print('Failed Task, Number of hole insertion events = ', len(insertion_events), '/2')
            else:
                event0 = insertion_events[0]
                event1 = insertion_events[1]
                if event0.hole_type is NCE.hole_type and event0.hole_idx == NCE.hole_idx:
                    if event1.hole_type is HoleType.ENTRY and event1.hole_idx == NCE.hole_idx:
                        self._report.success = True
                        self._report.entry_exit_idx = NCE.hole_idx
                        self._report.L_ntINexit_axial = ContactEventHelper.compute_axial_distance_from_hole(NCE.T_ntINhole)
                        self._report.L_ntINexit_lateral = ContactEventHelper.compute_lateral_distance_from_hole(event0.T_ntINhole)
                        self._report.L_ntINentry_lateral = ContactEventHelper.compute_lateral_distance_from_hole(event1.T_ntINhole)
                        self._report.P_max_ntINexit_lateral = ContactEventHelper.compute_max_lateral_component_from_hole(event0.T_ntINhole)
                        self._report.P_max_ntINentry_lateral = ContactEventHelper.compute_max_lateral_component_from_hole(event1.T_ntINhole)
                        self._report.P_ntINexit_lateral = event0.T_ntINhole.p
                        self._report.P_ntINentry_lateral = event1.T_ntINhole.p
                    else:
                        # Failed
                        print('Failed Task, Entry hole type / idx mismatch from closest type / idx')
                        print('Closest Type: ', NCE.hole_type, ' Idx: ', NCE.hole_idx)
                        print('Event1 Type: ', event1.hole_type, ' Idx: ', event1.hole_idx)
                else:
                    # Failed
                    print('Failed Task, Exit hole type / idx mismatch from closest type / idx')
                    print('Closest Type: ', NCE.hole_type, ' Idx: ', NCE.hole_idx)
                    print('Event0 Type: ', event0.hole_type, ' Idx: ', event0.hole_idx)
        else:
            print('Failed Task, The closest hole to needle tip and report completion is not of type ', HoleType.EXIT)

        self._report.print_report()


class Task_3_Evaluation_Report():
    def __init__(self):
        """

        """
        self.team_name = None

        # Needle protruding from the exit as the end of task
        self.L_ntINexit_axial = None

        # Cross-sectional distance from the exit hole's center
        self.L_ntINexit_lateral = [None for _ in range(GlobalParams.hole_count)]

        # Cross-sectional distance from the entry hole's center
        self.L_ntINentry_lateral = [None for _ in range(GlobalParams.hole_count)]

        self.entry_exit_idx = None

        self.completion_time = None

        self.success = False

    def print_report(self):
        """

        :return:
        """
        print('Team: ', self.team_name, ' Task 3 Completion Report: ')
        if self.success:
            print(OK_STR('\t Task Successful: '))
            print('\t Completion Time: ', self.completion_time)
            print('\t Needle Tip Axial Distance From Exit Hole (4/4) (Recommended {})'.format(GlobalParams.hole_bounds[0]), self.L_ntINexit_axial)
            for hidx in range(GlobalParams.hole_count):
                print('--------------------------------------------')
                print('\t Hole Number: ', hidx + 1, '/', GlobalParams.hole_count)
                print('\t Needle Tip Lateral Distance From Entry Hole During Insertion (Lower is Better): ',
                      self.L_ntINentry_lateral[hidx])
                print('\t Needle Tip Lateral Distance From Exit Hole During Insertion (Lower is Better): ',
                      self.L_ntINexit_lateral[hidx])
        else:
            print(FAIL_STR('Task Failed: '))


class Task_3_Evaluation():
    def __init__(self, client, team_name):
        """

        :param client:
        :param team_name:
        :return:
        """
        self._world = client.get_world_handle()
        self._needle_kinematics = NeedleKinematics()
        self._hole_objs = dict()
        self._hole_objs[HoleType.ENTRY] = []
        self._hole_objs[HoleType.EXIT] = []
        for i in range(GlobalParams.hole_count):
            self._hole_objs[HoleType.ENTRY].append(client.get_obj_handle("Entry" + str(i+1)))
            self._hole_objs[HoleType.EXIT].append(client.get_obj_handle("Exit" + str(i+1)))

        self._scene_trajectories = deque()
        self._needle_holes_proximity_events = dict()
        self._needle_holes_proximity_events[HoleType.ENTRY] = [deque() for _ in range(GlobalParams.hole_count)]
        self._needle_holes_proximity_events[HoleType.EXIT] = [deque() for _ in range(GlobalParams.hole_count)]

        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + team_name
        self._task_sub = rospy.Subscriber(prefix + '/task3/', Bool, self.task_completion_cb, queue_size=1)

        self._done = False
        self._report = Task_3_Evaluation_Report()
        self._report.team_name = team_name
        self._entry_exit_idx = -1
        self._start_time = rospy.Time.now().to_sec()
        time.sleep(1.0)

    def capture_scene_kinematics(self):
        """

        :return:
        """
        SKF = SceneKinematicsFrame()
        SKF.t = self._world._state.sim_time
        SKF.T_ntINw = self._needle_kinematics.get_tip_pose()

        for hole_type in HoleType:
            for i in range(GlobalParams.hole_count):
                SKF.T_holesINw[hole_type][i] = units_conversion.get_pose(self._hole_objs[hole_type][i])

        self._scene_trajectories.append(SKF)
        return SKF

    def compute_needle_hole_proximity_event(self, SKF):
        """

        :param SKF:
        :return:
        """
        proximity_events = []
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                T_ntINhole = SKF.T_holesINw[hole_type][hidx].Inverse() * SKF.T_ntINw
                ne = NeedleContactEvent()
                if ne.is_needle_intersecting_with_hole(T_ntINhole):
                    ne.hole_type = hole_type
                    ne.hole_idx = hidx
                    ne.T_ntINhole = T_ntINhole
                    ne.t = SKF.t
                    # ContactEventHelper.validate_needle_event(hole_type, hidx, ne)
                    (self._needle_holes_proximity_events[hole_type][hidx]).append(ne)
                    proximity_events.append(ne)
                    # print('\t\t', ne.hole_type, ne.hole_idx, ne.T_ntINhole.p.Norm())
        return proximity_events

    def task_completion_cb(self, msg):
        """

        :param msg:
        :return:
        """
        if msg.data is False:
            print('Error!, Task 3 Completion Message must be True')

        self._report.completion_time = rospy.Time.now().to_sec() - self._start_time
        self._done = True

    def evaluate(self):
        """

        :return:
        """
        t = 0.0
        while not self._done:
            time.sleep(0.01)
            SKF = self.capture_scene_kinematics()
            self.compute_needle_hole_proximity_event(SKF)
            t = t + 0.01
            if t % 1.0 >= 0.99:
                print(time.time(), ' ) Waiting for task completion report')

        # Record the final trajectories
        SKF = self.capture_scene_kinematics()
        print('Completion Report Submitted, Running evaluation')

        NCE = SKF.find_closest_hole_to_needle_tip()

        # self.validate_needle_insertion_events()

        self._report.success = False # Initialize to false
        if NCE.hole_type is HoleType.EXIT:
            insertion_events = ContactEventHelper.compute_insertion_events_from_proximity_events(
                self._needle_holes_proximity_events)
            if len(insertion_events) < 8:
                # Failed
                print('Failed Task, Number of hole insertion events =', len(insertion_events), 'out of 8')
                for ie in insertion_events:
                    print('\t Successful insertion into', ie.hole_type, ie.hole_idx)
            else:
                self._report.L_ntINexit_axial = ContactEventHelper.compute_axial_distance_from_hole(
                    NCE.T_ntINhole)
                self._report.success = True
                correct_idx = GlobalParams.hole_count
                for hidx in range(GlobalParams.hole_count):
                    event0 = insertion_events[2*hidx]
                    event1 = insertion_events[2*hidx+1]
                    correct_idx = correct_idx - 1
                    if event0.hole_type is HoleType.EXIT and event0.hole_idx == correct_idx:
                        if event1.hole_type is HoleType.ENTRY and event1.hole_idx == correct_idx:
                            self._report.L_ntINexit_lateral[hidx] = ContactEventHelper.compute_lateral_distance_from_hole(
                                event0.T_ntINhole)
                            self._report.L_ntINentry_lateral[hidx] = ContactEventHelper.compute_lateral_distance_from_hole(
                                event1.T_ntINhole)
                        else:
                            # Failed
                            print('Failed Task, Entry hole type / idx mismatch from closest type / idx')
                            print('Closest Type: ', NCE.hole_type, ' Idx: ', correct_idx)
                            print('Event1 Type: ', event1.hole_type, ' Idx: ', event1.hole_idx)
                            self._report.success = False
                    else:
                        # Failed
                        print('Failed Task, Exit hole type / idx mismatch from closest type / idx')
                        print('Closest Type: ', NCE.hole_type, ' Idx: ', correct_idx)
                        print('Event0 Type: ', event0.hole_type, ' Idx: ', event0.hole_idx)
                        self._report.success = False

        else:
            print('Failed Task, The closest hole to needle tip and report completion is not of type ', HoleType.EXIT)

        self._report.print_report()


def evaluate(args):
    client = Client('surgical_robotics_task_evaluation')
    client.connect()

    team_name = args.team_name
    task_to_evaluate = int(args.task_evaluation)
    if task_to_evaluate not in [1, 2, 3]:
        raise Exception('ERROR! Acceptable task evaluation options (-e option) are 1, 2 or 3')

    task_eval = None
    if task_to_evaluate == 1:
        task_eval = Task_1_Evaluation(client, team_name)
    elif task_to_evaluate == 2:
        task_eval = Task_2_Evaluation(client, team_name)
    elif task_to_evaluate == 3:
        task_eval = Task_3_Evaluation(client, team_name)

    task_eval.evaluate()
    print(OK_STR('GOOD BYE'))


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-t', action='store', dest='team_name', help='Team Name', default='test_team')
    parser.add_argument('-e', action='store', dest='task_evaluation', help='Task to evaluate (1,2 or 3)')

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)
    evaluate(parsed_args)

