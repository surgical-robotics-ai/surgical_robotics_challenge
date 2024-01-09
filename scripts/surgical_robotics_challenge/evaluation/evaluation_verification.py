import time

from ambf_client import Client
from PyKDL import Frame, Rotation, Vector
import rospy
from geometry_msgs.msg import PoseStamped
from surgical_robotics_challenge import units_conversion
from surgical_robotics_challenge.utils.utilities import *

from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.utils.interpolation import Interpolation
from surgical_robotics_challenge.evaluation.evaluation import NeedleKinematics
from surgical_robotics_challenge import task_completion_report
import math
from argparse import ArgumentParser


evaluation_prefix = '/surgical_robotics_challenge/completion_report/'

pi_2 = math.pi / 2.0

first_last_lin_hole_offset = Frame(Rotation.RPY(0, 0, 0), Vector(0., 0., 0.04))
entry_hole_offset = Frame(Rotation.RPY(-pi_2, pi_2, 0), Vector(0, 0, 0))
exit_hole_offset = Frame(Rotation.RPY(0, -pi_2, -pi_2), Vector(0, 0, 0))


def task_1_verification(simulaiton_manager, team_name):
    time.sleep(0.5)
    report = task_completion_report.TaskCompletionReport(team_name)
    n = simulaiton_manager.get_obj_handle('Needle')
    e = simulaiton_manager.get_obj_handle('CameraFrame')
    time.sleep(0.5)
    T_n_w = n.get_pose()
    T_e_w = e.get_pose()
    T_n_e = T_e_w.Inverse() * T_n_w
    T_n_e.M.GetQuaternion()

    time.sleep(0.5)
    report.task_1_report(frame_to_pose_stamped(T_n_e))


def move_needle_through_holes(simulation_manager, num_holes):
    world = simulation_manager.get_world_handle()
    needle = simulation_manager.get_obj_handle('Needle')
    entry_holes = []
    exit_holes = []
    for i in range(num_holes):
        entry_holes.append(simulation_manager.get_obj_handle('Entry' + str(i + 1)))
        exit_holes.append(simulation_manager.get_obj_handle('Exit' + str(i + 1)))

    time.sleep(0.3)
    world.reset_bodies()
    time.sleep(1.0)

    start_pose = needle.get_pose()
    pose_control_points = [start_pose]
    for i in range(num_holes):
        pose_control_points.append(entry_holes[i].get_pose() * first_last_lin_hole_offset * entry_hole_offset)
        pose_control_points.append(entry_holes[i].get_pose() * entry_hole_offset)
        pose_control_points.append(exit_holes[i].get_pose() * exit_hole_offset)
        pose_control_points.append(exit_holes[i].get_pose() * first_last_lin_hole_offset * exit_hole_offset)

    interpolater = Interpolation()

    T_nINt = NeedleKinematics.T_tINn.Inverse()

    for i in range(len(pose_control_points) - 1):
        p1 = frame_to_pose_vec(pose_control_points[i])
        p2 = frame_to_pose_vec(pose_control_points[i + 1])
        v1 = [0, 0, 0, 0, 0, 0]
        v2 = [0, 0, 0, 0, 0, 0]
        a1 = [0, 0, 0, 0, 0, 0]
        a2 = [0, 0, 0, 0, 0, 0]
        t1 = 0.
        t2 = 1.
        interpolater.compute_interpolation_params(p1, p2, v1, v2, a1, a2, t1, t2)
        start_time = time.time()
        delta_t = 0.
        while delta_t < t2:
            delta_t = time.time() - start_time
            if delta_t >= t2:
                break
            pt = interpolater.get_interpolated_x(delta_t)
            needle.set_pose(pose_vec_to_frame(pt) * T_nINt)
            time.sleep(0.01)
        time.sleep(2.0)


def tast_2_verification(simulation_manager, team_name):
    report = task_completion_report.TaskCompletionReport(team_name)
    move_needle_through_holes(simulation_manager, 1)
    report.task_2_report(True)


def tast_3_verification(simulation_manager, team_name):
    report = task_completion_report.TaskCompletionReport(team_name)
    move_needle_through_holes(simulation_manager, 4)
    report.task_3_report(True)


def frame_to_pose_vec(F):
    return [F.p[0], F.p[1], F.p[2], F.M.GetRPY()[0], F.M.GetRPY()[1], F.M.GetRPY()[2]]


def pose_vec_to_frame(v):
    return Frame(Rotation.RPY(v[3], v[4], v[5]),
                 Vector(v[0], v[1], v[2]))


def verification(args):
    simulation_manager = SimulationManager('surgical_robotics_challenge_verification')

    team_name = args.team_name
    task_to_evaluate = int(args.task_evaluation)
    if task_to_evaluate not in [1, 2, 3]:
        raise Exception('ERROR! Acceptable task evaluation options (-e option) are 1, 2 or 3')

    if task_to_evaluate == 1:
        task_1_verification(simulation_manager, team_name)
    elif task_to_evaluate == 2:
        tast_2_verification(simulation_manager, team_name)
    elif task_to_evaluate == 3:
        tast_3_verification(simulation_manager, team_name)

    print(OK_STR('GOOD BYE'))


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-t', action='store', dest='team_name', help='Team Name', default='test_team')
    parser.add_argument('-e', action='store', dest='task_evaluation', help='Task to evaluate (1 or 2)')

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)
    verification(parsed_args)
