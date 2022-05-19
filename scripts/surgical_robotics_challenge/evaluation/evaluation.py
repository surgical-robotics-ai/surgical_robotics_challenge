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
        """

        """
        self.team_name = None

        # Needle protruding from the exit as the end of task
        self.L_ntINexit_axial = None

        # Cross-sectional distance from the exit hole's center
        self.L_ntINexit_lateral = None

        # Cross-sectional distance from the entry hole's center
        self.L_ntINentry_lateral = None

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
            print('\t Needle Tip Axial Distance From Exit Hole (Lower is Better): ', self.L_ntINexit_axial)
            print('\t Needle Tip Lateral Distance From Exit Hole (Lower is Better): ', self.L_ntINexit_lateral)
            print('\t Needle Tip Lateral Distance From Entry Hole (Lower is Better): ', self.L_ntINentry_lateral)
        else:
            print(FAIL_STR('Task Failed: '))


class HoleType(Enum):
    """

    """
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
        self.T_holesINw[HoleType.ENTRY] = [Frame() for _ in range(self._hole_count)]
        self.T_holesINw[HoleType.EXIT] = [Frame() for _ in range(self._hole_count)]
        self.T_ntINw = Frame()
        self.t = 0.0

    def find_closest_hole_to_needle_tip(self):
        NCE = NeedleContactEvent()
        min_distance = 10000.
        for hole_type in HoleType:
            for hidx in range(self._hole_count):
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
        # The Object Aligned Bounding Box to check for needle tip
        self._hole_bounds = Vector(0.05, 0.05, 0.05)
        self.insertion_depth_threshold = 0.01

    def is_needle_intersecting_with_hole(self, T_ntINhole):
        """

        :param T_ntINhole:
        :return:
        """
        for j in range(3):
            if abs(T_ntINhole.p[j]) > self._hole_bounds[j]:
                # Needle tip is out of bounds, ignore
                return False
        return True


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
        self._needle_holes_proximity_events[HoleType.ENTRY] = [deque() for _ in range(self._hole_count)]
        self._needle_holes_proximity_events[HoleType.EXIT] = [deque() for _ in range(self._hole_count)]

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
        self._start_time = self._world._state.sim_time
        time.sleep(1.0)

    def compute_axial_distance_from_hole(self, T_ntINhole):
        """

        :return:
        """
        # The axial distance is along the z axes
        return abs(T_ntINhole.p[2])

    def compute_lateral_distance_from_hole(self, T_ntINhole):
        """

        :return:
        """
        p = T_ntINhole.p
        p[2] = 0.
        return p.Norm()

    def capture_scene_kinematics(self):
        """

        :return:
        """
        SKF = SceneKinematicsFrame(self._hole_count)
        SKF.t = self._world._state.sim_time
        SKF.T_ntINw = self._needle_kinematics.get_tip_pose()

        for hole_type in HoleType:
            for i in range(self._hole_count):
                SKF.T_holesINw[hole_type][i] = ambf_obj_pose_to_frame(self._hole_objs[hole_type][i])

        self._scene_trajectories.append(SKF)
        return SKF

    def compute_needle_hole_proximity_event(self, SKF):
        """

        :param SKF:
        :return:
        """
        proximity_events = []
        for hole_type in HoleType:
            for hidx in range(self._hole_count):
                T_ntINhole = SKF.T_holesINw[hole_type][hidx].Inverse() * SKF.T_ntINw
                ne = NeedleContactEvent()
                if ne.is_needle_intersecting_with_hole(T_ntINhole):
                    ne.hole_type = hole_type
                    ne.hole_idx = hidx
                    ne.T_ntINhole = T_ntINhole
                    ne.t = SKF.t
                    self.validate_needle_event(hole_type, hidx, ne)
                    (self._needle_holes_proximity_events[hole_type][hidx]).append(ne)
                    proximity_events.append(ne)
                    # print('\t\t', ne.hole_type, ne.hole_idx, ne.T_ntINhole.p.Norm())
        return proximity_events

    def validate_needle_event(self, hole_type, hole_idx, NE, print_output=True):
        if NE.hole_type != hole_type or NE.hole_idx != hole_idx:
            if print_output:
                print('ERROR! For hole_type: ', hole_type, ' and hole_idx: ' , hole_idx,
                      ' NE hole_type: ', NE.hole_type, ' hole_idx: ', NE.hole_idx)
            return False
        else:
            return True

    def validate_needle_insertion_events(self):
        incorrect_events = 0
        total_events = 0
        for hole_type in HoleType:
            for hidx in range(self._hole_count):
                event_count = len(self._needle_holes_proximity_events[hole_type][hidx])
                for e in range(event_count):
                    NE = self._needle_holes_proximity_events[hole_type][hidx][e]
                    total_events = total_events + 1
                    if not self.validate_needle_event(hole_type, hidx, NE, print_output=False):
                        incorrect_events = incorrect_events + 1
        print('Total Events: ', total_events, ' Incorrect Events: ', incorrect_events)

    def compute_insertion_events_from_proximity_events(self):
        hole_insertion_events = []
        for hole_type in HoleType:
            for hidx in range(self._hole_count):
                event_size = len(self._needle_holes_proximity_events[hole_type][hidx])
                if event_size == 0:
                    i_nearest_to_origin = -1
                elif event_size == 1:
                    i_nearest_to_origin = 0
                else:
                    z1 = abs(self._needle_holes_proximity_events[hole_type][hidx][1].T_ntINhole.p[2])
                    z0 = abs(self._needle_holes_proximity_events[hole_type][hidx][0].T_ntINhole.p[2])
                    if z1 < z0:
                        i_nearest_to_origin = 1
                        z_min_abs = z1
                    else:
                        i_nearest_to_origin = 0
                        z_min_abs = z0

                    for ev in range(2, event_size):
                        z = abs(self._needle_holes_proximity_events[hole_type][hidx][ev].T_ntINhole.p[2])
                        if z <= z_min_abs: # Using lte instead of le for comparison to find the latest time in the queue
                            z_min_abs = z
                            i_nearest_to_origin = ev
                if i_nearest_to_origin != -1:
                    NCE = self._needle_holes_proximity_events[hole_type][hidx][i_nearest_to_origin]
                    self.validate_needle_event(hole_type, hidx, NCE, print_output=True)
                    if abs(NCE.T_ntINhole.p[2]) < NCE.insertion_depth_threshold:
                        hole_insertion_events.append(NCE)
        # Sort the list based on time events
        hole_insertion_events.sort(key=lambda x: x.t, reverse=True)
        return hole_insertion_events

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
            SKF = self.capture_scene_kinematics()
            self.compute_needle_hole_proximity_event(SKF)
            t = t + 0.01
            if t % 1.0 >= 0.99:
                print(time.time(), ' ) Waiting for task 2 completion report')

        # Record the final trajectories
        SKF = self.capture_scene_kinematics()
        print('Completion Report Submitted, Running evaluation')

        NCE = SKF.find_closest_hole_to_needle_tip()

        # self.validate_needle_insertion_events()

        self._report.success = False # Initialize to false
        if NCE.hole_type is HoleType.EXIT:
            self._report.completion_time = self._completion_time
            insertion_events = self.compute_insertion_events_from_proximity_events()
            if len(insertion_events) < 2:
                # Failed
                pass
            else:
                event0 = insertion_events[0]
                event1 = insertion_events[1]
                if event0.hole_type is NCE.hole_type and event0.hole_idx == NCE.hole_idx:
                    if event1.hole_type is HoleType.ENTRY and event1.hole_idx == NCE.hole_idx:
                        self._report.success = True
                        self._report.entry_exit_idx = NCE.hole_idx
                        self._report.L_ntINexit_axial = self.compute_axial_distance_from_hole(NCE.T_ntINhole)
                        self._report.L_ntINexit_lateral = self.compute_lateral_distance_from_hole(event0.T_ntINhole)
                        self._report.L_ntINentry_lateral = self.compute_lateral_distance_from_hole(event1.T_ntINhole)
                else:
                    # Failed
                    pass
        else:
            # Failed
            pass

        self._report.print_report()
