from PyKDL import Frame, Vector, Wrench
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.teleoperation.input_devices.mtm_device_crtk import TeleopMode
from surgical_robotics_challenge.utils import coordinate_frames, utilities


def load_selected_psm_arms(simulation_manager, cam, run_psm_one, run_psm_two, run_psm_three, tool_id=None):
    psm_by_name = {}

    psm_configs = [
        ('psm1', run_psm_one, coordinate_frames.PSM1.T_tip_cam),
        ('psm2', run_psm_two, coordinate_frames.PSM2.T_tip_cam),
        ('psm3', run_psm_three, coordinate_frames.PSM3.T_tip_cam),
    ]

    for arm_name, enabled, tip_in_camera in psm_configs:
        if not enabled:
            continue

        print('LOADING CONTROLLER FOR ', arm_name)
        psm_kwargs = {'add_joint_errors': False}
        if tool_id is not None:
            psm_kwargs['tool_id'] = tool_id

        psm = PSM(simulation_manager, arm_name, **psm_kwargs)
        if not psm.is_present():
            continue

        T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * tip_in_camera
        psm.set_home_pose(T_psmtip_b)
        psm_by_name[arm_name] = psm

    return psm_by_name


def _clamp(value, lower, upper):
    return max(lower, min(value, upper))


def compute_mtm_pose_error_wrench(
    measured_cp,
    servo_cp,
    measured_cv,
    kp_pos=120.0,
    kd_pos=0.0,
    kp_rot=2.5,
    kd_rot=0.05,
    max_force=2.0,
    max_torque=0.0,
    linear_deadband=0.001,
    angular_deadband=0.01,
):
    pose_error = servo_cp.Inverse() * measured_cp
    pos_err = servo_cp.M * pose_error.p
    rot_err = pose_error.M.GetRPY()

    if abs(pos_err[0]) < linear_deadband:
        pos_err[0] = 0.0
    if abs(pos_err[1]) < linear_deadband:
        pos_err[1] = 0.0
    if abs(pos_err[2]) < linear_deadband:
        pos_err[2] = 0.0

    rx, ry, rz = rot_err
    if abs(rx) < angular_deadband:
        rx = 0.0
    if abs(ry) < angular_deadband:
        ry = 0.0
    if abs(rz) < angular_deadband:
        rz = 0.0

    fx = _clamp(kp_pos * pos_err[0] - kd_pos * measured_cv.vel[0], -max_force, max_force)
    fy = _clamp(kp_pos * pos_err[1] - kd_pos * measured_cv.vel[1], -max_force, max_force)
    fz = _clamp(kp_pos * pos_err[2] - kd_pos * measured_cv.vel[2], -max_force, max_force)

    tx = _clamp(kp_rot * rx - kd_rot * measured_cv.rot[0], -max_torque, max_torque)
    ty = _clamp(kp_rot * ry - kd_rot * measured_cv.rot[1], -max_torque, max_torque)
    tz = _clamp(kp_rot * rz - kd_rot * measured_cv.rot[2], -max_torque, max_torque)

    return Wrench(Vector(fx, fy, fz), Vector(tx, ty, tz))


def apply_mtm_to_psm_command(
    leader,
    psm,
    T_c_b,
    update_dt,
    set_jaw_only_when_coag=False,
    enable_force_feedback=False,
):
    mode = TeleopMode.HOLD
    if leader.clutch_button_pressed:
        mode = TeleopMode.CLUTCH
    elif leader.coag_button_pressed and not leader.clutch_button_pressed:
        mode = TeleopMode.COAG

    leader.set_teleop_mode(mode)

    measured_cp = leader.measured_cp()
    measured_cv = leader.measured_cv()
    twist = measured_cv * coordinate_frames.TeleopScale.scale_factor

    cmd_xyz = psm.T_t_b_home.p
    if not leader.clutch_button_pressed:
        delta_t = T_c_b.M * twist.vel * update_dt
        cmd_xyz = cmd_xyz + delta_t
        psm.T_t_b_home.p = cmd_xyz

    T_ik = None
    if leader.get_teleop_mode() == TeleopMode.COAG:
        cmd_rpy = T_c_b.M * measured_cp.M
        T_ik = Frame(cmd_rpy, cmd_xyz)
        psm.servo_cp(T_ik)

    measured_psm_cp = utilities.convert_mat_to_frame(psm.measured_cp())
    # Use the active slave-space command as the force reference.
    servo_cp_target = T_ik if T_ik is not None else measured_psm_cp

    if enable_force_feedback and leader.get_teleop_mode() == TeleopMode.COAG:
        wrench_kwargs = leader.get_pose_error_wrench_params()
        wrench = compute_mtm_pose_error_wrench(
            measured_psm_cp,
            servo_cp_target,
            measured_cv,
            **wrench_kwargs,
        )
        leader.servo_cf(T_c_b.M.Inverse() * wrench)

    if (not set_jaw_only_when_coag) or leader.get_teleop_mode() == TeleopMode.COAG:
        psm.set_jaw_angle(leader.get_jaw_angle())

    return cmd_xyz, T_ik