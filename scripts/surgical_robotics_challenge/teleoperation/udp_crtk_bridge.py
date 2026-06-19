#!/usr/bin/env python3
"""
udp_crtk_bridge.py

A UDP <-> CRTK-ROS bridge that makes AMBF look like a dVRK `sawSocketStreamer`
to the Meta Quest teleoperation app. The Quest cannot tell whether it is talking
to a real robot's sawSocketStreamer or to this bridge: same wire format, same
verbs, same frames.

Wire protocol (reverse-engineered from the Quest's UDPComm.cs / MovementController):
  Bridge SENDS  (bridge -> Quest, what the Quest parses):
    PSM pose : {"setpoint_cp": {"Position": {"Rotation": [[3x3]], "Translation":[x,y,z]}, "Timestamp":t, "Valid":true, "AutomaticTimestamp":true}}
    PSM jaw  : {"jaw/setpoint_js": {"Name":["jaw"], "Position":[angle], "Effort":[0], "Velocity":[0], "Timestamp":t, "Valid":true, "AutomaticTimestamp":true}}
    ECM pose : {"setpoint_cp": {...}}                       (same cp shape)
    ECM joints:{"setpoint_js": {"Name":["yaw","pitch","insertion","roll"], "Position":[4], ...}}
  Bridge RECEIVES (Quest -> bridge):
    PSM pose : {"move_cp":{"Goal":{"Rotation":[[3x3]],"Translation":[x,y,z]}}}
    PSM jaw  : {"jaw/move_jp":{"Goal":[angle]}}
    ECM joints:{"move_jp":{"Goal":[yaw,pitch,insertion,roll]}}

Frames / units: PSM cp is tool-tip-in-PSM-base-frame, SI meters (psm_arm.measured_cp
returns FK in base frame; servo_cp runs IK on a tip-in-base frame). The bridge does
NOT swap axes or rescale -- it passes base-frame poses straight through, exactly as a
real sawSocketStreamer would. Any constant base-frame offset is handled by your
external calibration. If orientation comes out mirrored, flip TRANSPOSE_ROTATION_OUT.

Ports (from UDPComm.cs): the Quest binds its receive sockets on 48051 (PSM1),
48052 (PSM2), 48053 (ECM), and its senders use 5701/5702/5703. So the bridge sends
state to QUEST_IP:4805x and binds 570x to receive commands (matching the real-robot
convention so the Quest's pre-set remote endpoints line up). The Quest will not send
anything until it has received PSM setpoints, so the bridge must know QUEST_IP up
front (no "reply to sender" -- that would deadlock).

ECM note: /CRTK/ecm/measured_js is hardcoded zeros (ecm_arm.py never updates
_measured_jp), so the bridge ECHOES the last commanded ECM joint vector back as
setpoint_js. This is also the hook for the future fixed-camera (no-ECM) setup.

Prerequisites (two terminals before this one):
  T1:  ./ambf_simulator --launch_file <srch>/launch.yaml -l 0,1,3,4,14,15 -p 120 -t 1
  T2:  python3 <srch>/scripts/surgical_robotics_challenge/launch_crtk_interface.py
"""

import argparse
import json
import math
import socket

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# Local IK (optional): only needed for CMD_OUTPUT="servo_jp". Imported lazily-safe so
# the bridge still runs in servo_cp mode on a machine without the package / PyKDL.
try:
    from PyKDL import Frame, Rotation, Vector
    from surgical_robotics_challenge.kinematics.psmKinematics import (
        PSMKinematicSolver, enforce_limits,
    )
    _IK_AVAILABLE = True
except Exception as _ik_err:  # ImportError, or PyKDL missing, etc.
    _IK_AVAILABLE = False
    _IK_IMPORT_ERROR = _ik_err

# ============================ CONFIG =========================================
QUEST_IP = "127.0.0.1"     # destination for outbound state (the headset's IP).
BIND_IP  = "0.0.0.0"       # local bind for the command-receive side.

# Per-arm: (port the bridge sends state FROM / receives commands ON, Quest's listen port)
PORTS = {
    "psm1": {"bind": 5701, "quest": 48051},
    "psm2": {"bind": 5702, "quest": 48052},
    "ecm":  {"bind": 5703, "quest": 48053},
}

SEND_HZ = 60.0             # outbound state rate
RECV_POLL_HZ = 200.0       # how often we drain inbound command datagrams

# If the Quest shows a mirrored/transposed orientation, set this True first.
TRANSPOSE_ROTATION_OUT = False

# ---- Camera-frame offset rotation (the tunable knob) ------------------------
# On a real dVRK the setup joints supply the camera<->PSM-base transform; with them
# removed we approximate it as a pure ROTATION and tune it by hand. This rotation is
# applied to ALL PSM cp data, consistently both directions:
#   incoming (Quest move_cp, in command/camera frame) -> base frame:  R_OFFSET
#   outgoing (AMBF measured_cp, in base frame) -> command/camera frame: R_OFFSET^T
# Because the two are exact inverses, the round trip is identity (a pose commanded and
# read back is unchanged), so the Quest's relative teleop stays consistent; only the
# *direction* that hand motion maps to is rotated -- which is the whole point.
# Translation is rotated too (motion direction is the goal); the camera<->base
# translation offset is intentionally ignored and washes out under relative teleop.
# Units are DEGREES for easy guessing. (0,0,0) = no change = previous behavior.
# Convention matches PyKDL Rotation.RPY: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
CP_OFFSET_RPY_DEG = (180.0, 0.0, 180.0)   # (roll, pitch, yaw) quest/camera -> base

# Swap which physical PSM each Quest channel drives. Quest "psm1" (ports 5701/48051)
# then drives AMBF psm2 and vice-versa, feedback included. Use when the sim's left/right
# arrangement is mirrored vs. the operator's expectation.
SWAP_PSMS = False

# How the bridge turns an incoming Quest pose into a robot command:
#   "servo_cp" : publish the pose to servo_cp; the CRTK interface runs IK (current,
#                works in AMBF). Requires the target to implement servo_cp.
#   "servo_jp" : run the closed-form IK HERE (compute_IK) and publish joints to
#                servo_jp. For targets that don't implement servo_cp (e.g. IsaacSim).
#                Requires the surgical_robotics_challenge package importable (PyKDL).
# Testing in AMBF: set this to "servo_jp" and the arm should behave IDENTICALLY to
# "servo_cp" -- same compute_IK, same frames, just run one layer earlier. That identity
# is the validation that local IK is correct before pointing it at IsaacSim.
CMD_OUTPUT = "servo_cp"   # "servo_cp" or "servo_jp"

# Tool id for the local IK solver (selects the per-tool geometry from JSON).
# 420006 = LND_SI (the tool used in this project, per the tool_id_420006 bodies).
# 400006 = classic LND. The AMBF wrapper auto-detects this from the body name; the
# bridge can't, so set it explicitly to match your loaded tool.
TOOL_ID = 420006

# ---- Target profile: which ROS interface the bridge drives -------------------
#   "ambf"  : CRTK interface. Topics /CRTK/psm1/...  Has servo_cp (interface runs IK).
#   "isaac" : IsaacSim. Topics /PSM1/...  No servo_cp -> MUST run IK here (servo_jp).
# A profile fixes: the namespace, the per-arm topic name (lowercase psm1 vs UPPER PSM1),
# and whether servo_cp exists. Selecting "isaac" forces CMD_OUTPUT="servo_jp".
TARGET = "ambf"

# Per-profile arm topic name: internal key "psm1"/"psm2" -> the name used IN TOPICS.
_ARM_NAME = {
    "ambf":  {"psm1": "psm1", "psm2": "psm2"},
    "isaac": {"psm1": "PSM1", "psm2": "PSM2"},
}
# Per-profile topic templates. {arm} is filled with the profile arm name above.
_TOPICS = {
    "ambf": {
        "measured_cp": "/CRTK/{arm}/measured_cp",
        "servo_cp":    "/CRTK/{arm}/servo_cp",
        "servo_jp":    "/CRTK/{arm}/servo_jp",
        "jaw_servo":   "/CRTK/{arm}/jaw/servo_jp",
        "has_servo_cp": True,
        "has_ecm":     True,
        "ecm_measured_cp": "/CRTK/ecm/measured_cp",
        "ecm_servo_jp":    "/CRTK/ecm/servo_jp",
    },
    "isaac": {
        "measured_cp": "/{arm}/measured_cp",
        "servo_cp":    None,                       # IsaacSim has no servo_cp
        "servo_jp":    "/{arm}/servo_jp",
        "jaw_servo":   "/{arm}/jaw/servo_jp",
        "has_servo_cp": False,
        "has_ecm":     False,                      # no ECM in the IsaacSim topic list
        "ecm_measured_cp": None,
        "ecm_servo_jp":    None,
    },
}

ECM_JOINT_NAMES = ["yaw", "pitch", "insertion", "roll"]
DEFAULT_JAW = 0.5          # matches PSM.__init__ set_jaw_angle(0.5)

# Joint-name array populated on every PSM servo_jp message (homing + any local IK).
# Order matches the IK output [j1..j6] = [outer_yaw, outer_pitch, insertion,
# tool_roll, wrist_pitch, wrist_yaw]. The AMBF CRTK interface keys on position index
# (not name), so these are advisory there, but kept consistent with the IsaacSim build.
PSM_JOINT_NAMES = [
    "Base_Yaw",
    "Yaw_Pitch_End",
    "Pitch_End_Main_Insert",
    "Main_Insert_Tool_Roll",
    "Tool_Roll_Tool_Pitch",
    "Tool_Yaw_Tool_Pitch",
]

# ---- Startup homing (opt-in via --home) -------------------------------------
# With --home, the bridge BLASTS servo_jp at the home joint vector ~HOME_BURST_N times
# over ~HOME_BURST_S seconds before doing anything else, then bridges normally.
# servo_jp sets joints immediately (no trajectory); the repeat guarantees the command
# lands even if early datagrams drop before the subscriber connects.
# Homing is OFF by default so a re-run/crash-restart does NOT re-home a robot that
# is already positioned -- pass --home only on the first, intentional launch.
HOME_INSERTION_M = 0.12
# 6-joint home: [outer_yaw, outer_pitch, insertion, tool_roll, wrist_pitch, wrist_yaw]
HOME_JP = [0.0, 0.0, HOME_INSERTION_M, 0.0, 0.0, 0.0]
HOME_BURST_N = 30          # number of servo_jp messages to send
HOME_BURST_S = 0.5         # spread them over this many seconds (~60 Hz)
HOME_DISCOVERY_TIMEOUT_S = 10.0  # max wait for the CRTK servo_jp subscriber to connect
# =============================================================================


# ---- rotation <-> quaternion helpers ----------------------------------------
def quat_to_matrix(qx, qy, qz, qw):
    """Unit quaternion (x,y,z,w) -> 3x3 rotation matrix (numpy)."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def matrix_to_quat(R):
    """3x3 rotation matrix -> quaternion (x,y,z,w), Shepperd's method."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def rpy_deg_to_matrix(roll_deg, pitch_deg, yaw_deg):
    """(roll,pitch,yaw) in DEGREES -> 3x3 matrix. Matches PyKDL Rotation.RPY:
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# Precompute the offset rotation once. R_OFFSET maps command/camera-frame -> base-frame.
R_OFFSET = rpy_deg_to_matrix(*CP_OFFSET_RPY_DEG)


def offset_out(R, t):
    """base-frame pose -> command/camera-frame (for outgoing setpoint_cp). R_OFFSET^T."""
    Rt = R_OFFSET.T
    return Rt @ np.asarray(R), Rt @ np.asarray(t, dtype=float)


def offset_in(R, t):
    """command/camera-frame pose -> base-frame (for incoming move_cp). R_OFFSET."""
    return R_OFFSET @ np.asarray(R), R_OFFSET @ np.asarray(t, dtype=float)


# ---- JSON payload builders (match the sawSocketStreamer shape) --------------
def cp_payload(key, R, translation, stamp):
    # ORDER MATTERS: Rotation must come before Translation. The Quest's
    # JsonHelper.GetRotation stops at the first '"' after "Rotation":, so it relies
    # on Translation's key-quote to terminate the rotation array. Reordering breaks it.
    if TRANSPOSE_ROTATION_OUT:
        R = np.asarray(R).T
    return {
        key: {
            "AutomaticTimestamp": True,
            "Position": {
                "Rotation": [[float(R[i][j]) for j in range(3)] for i in range(3)],
                "Translation": [float(v) for v in translation],
            },
            "Timestamp": float(stamp),
            "Valid": True,
        }
    }


def js_payload(key, names, positions, stamp):
    n = len(positions)
    return {
        key: {
            "AutomaticTimestamp": True,
            "Effort": [0.0] * n,
            "Name": list(names),
            "Position": [float(v) for v in positions],
            "Timestamp": float(stamp),
            "Valid": True,
            "Velocity": [0.0] * n,
        }
    }


def stamp_to_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# ---- one bidirectional UDP socket bound per arm -----------------------------
class ArmSocket:
    def __init__(self, node, name, bind_port, quest_port):
        self.node = node
        self.name = name
        self.dest = (QUEST_IP, quest_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((BIND_IP, bind_port))
        self.sock.setblocking(False)

    def send_json(self, obj):
        try:
            # COMPACT separators are REQUIRED: the Quest's JsonHelper.JawAngle/Joints
            # parsers use index math that assumes "Position":[...] with no space after
            # the colon (matching real dVRK sawSocketStreamer output). A default
            # ", "/": " from json.dumps shifts their window and breaks float.Parse.
            data = json.dumps(obj, separators=(",", ":")).encode("utf-8")
            self.sock.sendto(data, self.dest)
        except OSError as e:
            self.node.get_logger().error(f"[{self.name}] send failed: {e}")

    def drain(self, handler):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as e:
                self.node.get_logger().error(f"[{self.name}] recv error: {e}")
                return
            text = data.decode("utf-8", errors="ignore").strip().strip("\x00")
            if not text:
                continue
            try:
                obj = json.loads(text)
            except json.JSONDecodeError as e:
                self.node.get_logger().warn(f"[{self.name}] bad JSON: {e}")
                continue
            handler(obj)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# ---- PSM: send setpoint_cp + jaw/setpoint_js; recv move_cp + jaw/move_jp -----
class PsmBridge:
    def __init__(self, node, udp_name, robot_name):
        # udp_name  : which Quest channel/ports this serves (psm1/psm2)
        # robot_name: which AMBF arm it actually drives (differs from udp_name if swapped)
        self.node = node
        self.udp_name = udp_name
        self.robot_name = robot_name
        self.arm = robot_name   # internal key / frame_id / logging use lowercase psmN
        self.io = ArmSocket(node, udp_name, PORTS[udp_name]["bind"], PORTS[udp_name]["quest"])

        self.latest_cp = None          # (R 3x3, translation, stamp)
        self.last_jaw = DEFAULT_JAW

        # Resolve topics for the active target profile (namespace + arm-name casing).
        prof = _TOPICS[TARGET]
        arm_t = _ARM_NAME[TARGET][robot_name]      # e.g. "psm1" (ambf) or "PSM1" (isaac)
        t_measured_cp = prof["measured_cp"].format(arm=arm_t)
        t_servo_jp    = prof["servo_jp"].format(arm=arm_t)
        t_jaw_servo   = prof["jaw_servo"].format(arm=arm_t)
        t_servo_cp    = prof["servo_cp"].format(arm=arm_t) if prof["has_servo_cp"] else None

        node.create_subscription(PoseStamped, t_measured_cp, self._on_measured_cp, 10)
        self.jaw_pub = node.create_publisher(JointState, t_jaw_servo, 10)
        self.servo_jp_pub = node.create_publisher(JointState, t_servo_jp, 10)
        self.cmd_cp_pub = (node.create_publisher(PoseStamped, t_servo_cp, 10)
                           if t_servo_cp is not None else None)
        swap_note = f" (Quest {udp_name} -> robot {robot_name})" if udp_name != robot_name else ""

        # Local-IK solver (only for CMD_OUTPUT="servo_jp").
        self.ik = None
        self._last_jp = None   # hold-last-good on IK failure
        if CMD_OUTPUT == "servo_jp":
            if not _IK_AVAILABLE:
                raise RuntimeError(
                    f"CMD_OUTPUT='servo_jp' needs the surgical_robotics_challenge IK "
                    f"(PyKDL + package), which failed to import: {_IK_IMPORT_ERROR}"
                )
            self.ik = PSMKinematicSolver(psm_type=TOOL_ID, tool_id=TOOL_ID)

        cmd_desc = (f"{t_servo_jp} via local IK (tool {TOOL_ID})" if CMD_OUTPUT == "servo_jp"
                    else t_servo_cp)
        node.get_logger().info(
            f"[{udp_name}{swap_note}] target={TARGET} send->{self.io.dest}  "
            f"recv:{PORTS[udp_name]['bind']}  (cp from {t_measured_cp}, cmd: {cmd_desc})"
        )

    def home_tick(self):
        """Publish HOME_JP once via servo_jp (called repeatedly during the home burst)."""
        js = JointState()
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.name = list(PSM_JOINT_NAMES)
        js.position = list(HOME_JP)
        self.servo_jp_pub.publish(js)

    def _on_measured_cp(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        R = quat_to_matrix(o.x, o.y, o.z, o.w)
        self.latest_cp = (R, (p.x, p.y, p.z), stamp_to_sec(msg))

    def on_send_tick(self, now_sec):
        if self.latest_cp is not None:
            R, t, stamp = self.latest_cp
            # base-frame -> command/camera-frame before sending to the Quest.
            R_out, t_out = offset_out(R, t)
            self.io.send_json(cp_payload("setpoint_cp", R_out, t_out, stamp))
        # jaw feedback: echo last commanded jaw (real interface doesn't publish jaw measured)
        self.io.send_json(js_payload("jaw/setpoint_js", ["jaw"], [self.last_jaw], now_sec))

    def poll(self):
        self.io.drain(self._handle)

    def _handle(self, obj):
        if "jaw/move_jp" in obj:
            goal = obj["jaw/move_jp"].get("Goal", [])
            if goal:
                self.last_jaw = float(goal[0])
                js = JointState()
                js.header.stamp = self.node.get_clock().now().to_msg()
                # js.name = ["jaw"]
                js.position = [self.last_jaw]
                self.jaw_pub.publish(js)
        elif "move_cp" in obj or "servo_cp" in obj:
            key = "move_cp" if "move_cp" in obj else "servo_cp"
            goal = obj[key].get("Goal", {})
            R = goal.get("Rotation")
            T = goal.get("Translation")
            if R is None or T is None:
                return
            # command/camera-frame -> base-frame (the frame both servo_cp and IK expect).
            R_base, T_base = offset_in(R, T)

            if CMD_OUTPUT == "servo_jp":
                self._publish_servo_jp(R_base, T_base)
            else:
                self._publish_servo_cp(R_base, T_base)

    def _publish_servo_cp(self, R_base, T_base):
        qx, qy, qz, qw = matrix_to_quat(R_base)
        cmd = PoseStamped()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.header.frame_id = f"{self.robot_name}/baselink"
        cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = (
            float(T_base[0]), float(T_base[1]), float(T_base[2])
        )
        cmd.pose.orientation.x = qx
        cmd.pose.orientation.y = qy
        cmd.pose.orientation.z = qz
        cmd.pose.orientation.w = qw
        # servo_cp: immediate IK in the CRTK interface (the Quest labels it move_cp, but
        # move_cp trajectories per-packet and fails at streaming rate, so we use servo_cp).
        self.cmd_cp_pub.publish(cmd)

    def _publish_servo_jp(self, R_base, T_base):
        # Run the SAME closed-form IK the CRTK interface would, but here, then publish
        # joints to servo_jp (for targets without servo_cp). Mirrors psm_arm.servo_cp:
        # compute_IK -> enforce_limits -> servo_jp. On IK failure, hold the last good
        # solution rather than publishing garbage.
        frame = Frame(
            Rotation(
                float(R_base[0][0]), float(R_base[0][1]), float(R_base[0][2]),
                float(R_base[1][0]), float(R_base[1][1]), float(R_base[1][2]),
                float(R_base[2][0]), float(R_base[2][1]), float(R_base[2][2]),
            ),
            Vector(float(T_base[0]), float(T_base[1]), float(T_base[2])),
        )
        try:
            jp = self.ik.compute_IK(frame)
            jp = enforce_limits(jp, self.ik.lower_limits, self.ik.upper_limits)
            if any(np.isnan(v) or np.isinf(v) for v in jp):
                raise ValueError("non-finite IK solution")
            self._last_jp = list(jp)
        except Exception as e:
            if self._last_jp is None:
                self.node.get_logger().warn(f"[{self.robot_name}] IK failed, no prior pose to hold: {e}")
                return
            self.node.get_logger().warn(f"[{self.robot_name}] IK failed, holding last pose: {e}")
            jp = self._last_jp

        js = JointState()
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.name = list(PSM_JOINT_NAMES)
        js.position = [float(v) for v in jp]
        self.servo_jp_pub.publish(js)

    def close(self):
        self.io.close()


# ---- ECM: send setpoint_cp + setpoint_js(echo); recv move_jp -> servo_jp -----
class EcmBridge:
    def __init__(self, node):
        self.node = node
        self.io = ArmSocket(node, "ecm", PORTS["ecm"]["bind"], PORTS["ecm"]["quest"])
        self.latest_cp = None
        self.last_joints = [0.0, 0.0, 0.0, 0.0]   # echoed back as setpoint_js

        node.create_subscription(PoseStamped, _TOPICS[TARGET]["ecm_measured_cp"], self._on_measured_cp, 10)
        self.servo_jp_pub = node.create_publisher(JointState, _TOPICS[TARGET]["ecm_servo_jp"], 10)
        node.get_logger().info(
            f"[ecm] send->{self.io.dest}  recv:{PORTS['ecm']['bind']}  "
            f"(cp from {_TOPICS[TARGET]['ecm_measured_cp']}, joints echoed; "
            f"cmd to {_TOPICS[TARGET]['ecm_servo_jp']})"
        )

    def _on_measured_cp(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        R = quat_to_matrix(o.x, o.y, o.z, o.w)
        self.latest_cp = (R, (p.x, p.y, p.z), stamp_to_sec(msg))

    def on_send_tick(self, now_sec):
        if self.latest_cp is not None:
            R, t, stamp = self.latest_cp
            self.io.send_json(cp_payload("setpoint_cp", R, t, stamp))
        # ECM joint feedback is echoed (AMBF measured_js is hardcoded zeros).
        self.io.send_json(js_payload("setpoint_js", ECM_JOINT_NAMES, self.last_joints, now_sec))

    def poll(self):
        self.io.drain(self._handle)

    def _handle(self, obj):
        if "move_jp" in obj or "servo_jp" in obj:
            key = "move_jp" if "move_jp" in obj else "servo_jp"
            goal = obj[key].get("Goal", [])
            if len(goal) >= 4:
                self.last_joints = [float(v) for v in goal[:4]]
                js = JointState()
                js.header.stamp = self.node.get_clock().now().to_msg()
                js.name = list(ECM_JOINT_NAMES)
                js.position = list(self.last_joints)
                # map move_jp -> servo_jp (ECM interface only subscribes servo_jp)
                self.servo_jp_pub.publish(js)

    def close(self):
        self.io.close()


class UdpCrtkBridge(Node):
    def __init__(self):
        super().__init__("udp_crtk_bridge")
        # Map each Quest channel to the AMBF arm it drives (swap flips the pairing).
        if SWAP_PSMS:
            pairing = [("psm1", "psm2"), ("psm2", "psm1")]
            self.get_logger().info("SWAP_PSMS=True: Quest psm1<->robot psm2, psm2<->psm1.")
        else:
            pairing = [("psm1", "psm1"), ("psm2", "psm2")]
        self.bridges = [PsmBridge(self, u, r) for (u, r) in pairing]
        if _TOPICS[TARGET]["has_ecm"]:
            self.bridges.append(EcmBridge(self))
        else:
            self.get_logger().info(f"target={TARGET}: no ECM in profile; ECM bridge skipped.")
        self.psm_bridges = [b for b in self.bridges if isinstance(b, PsmBridge)]
        if not np.allclose(R_OFFSET, np.eye(3)):
            self.get_logger().info(f"CP offset (deg, rpy)={CP_OFFSET_RPY_DEG} active on PSM cp.")
        self.create_timer(1.0 / SEND_HZ, self._on_send)
        self.create_timer(1.0 / RECV_POLL_HZ, self._on_recv)
        self.get_logger().info(f"Bridge up (target={TARGET}, output={CMD_OUTPUT}). State -> {QUEST_IP}.")

    def run_home_burst(self):
        """Blocking: wait for each PSM's servo_jp subscriber (the CRTK interface) to be
        DISCOVERED, then blast HOME_JP ~HOME_BURST_N times over ~HOME_BURST_S s. The wait
        is the fix for 'homing only works every few runs': ROS 2 drops published messages
        until the publisher<->subscriber match completes, and discovery latency is
        nondeterministic, so a fixed-duration blind burst sometimes finishes before the
        link is up. Done BEFORE the normal bridge loop -- nothing else runs during homing."""
        # 1) Wait until every PSM servo_jp publisher has at least one matched subscriber.
        deadline = self.get_clock().now().nanoseconds * 1e-9 + HOME_DISCOVERY_TIMEOUT_S
        pending = list(self.psm_bridges)
        while pending:
            rclpy.spin_once(self, timeout_sec=0.05)
            pending = [b for b in pending if b.servo_jp_pub.get_subscription_count() < 1]
            if not pending:
                break
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                names = ", ".join(b.robot_name for b in pending)
                self.get_logger().warn(
                    f"Homing: timed out after {HOME_DISCOVERY_TIMEOUT_S}s waiting for "
                    f"servo_jp subscriber(s) on [{names}]. Is launch_crtk_interface.py "
                    f"running? Bursting anyway (may not land)."
                )
                break
        if not pending:
            self.get_logger().info("Homing: servo_jp subscriber(s) connected.")

        # 2) Now the link is up -- blast HOME_JP.
        self.get_logger().info(
            f"Homing: blasting servo_jp {HOME_JP} x{HOME_BURST_N} over {HOME_BURST_S}s..."
        )
        dt = HOME_BURST_S / HOME_BURST_N
        for _ in range(HOME_BURST_N):
            for b in self.psm_bridges:
                b.home_tick()
            rclpy.spin_once(self, timeout_sec=dt)
        self.get_logger().info("Homing burst complete.")

    def _on_send(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for b in self.bridges:
            b.on_send_tick(now)

    def _on_recv(self):
        for b in self.bridges:
            b.poll()

    def close_all(self):
        for b in self.bridges:
            b.close()


def main():
    global QUEST_IP, CP_OFFSET_RPY_DEG, R_OFFSET, SWAP_PSMS, CMD_OUTPUT, TARGET
    parser = argparse.ArgumentParser()
    parser.add_argument("--quest-ip", default=QUEST_IP, help="Headset IP to send state to")
    parser.add_argument("--target", choices=["ambf", "isaac"], default=TARGET,
                        help="ambf: /CRTK/psm1 topics, has servo_cp. "
                             "isaac: /PSM1 topics, no servo_cp (forces --cmd-output servo_jp).")
    parser.add_argument("--home", action="store_true",
                        help=f"Blast servo_jp at HOME_JP {HOME_JP} x{HOME_BURST_N} at startup "
                             "(insertion off the singular zero pose), THEN bridge. "
                             "Omit on re-runs so a crash-restart does NOT re-home.")
    parser.add_argument("--offset-rpy", nargs=3, type=float, metavar=("ROLL", "PITCH", "YAW"),
                        help="Camera->base offset rotation in DEGREES (overrides "
                             f"CP_OFFSET_RPY_DEG={CP_OFFSET_RPY_DEG}).")
    parser.add_argument("--swap", action="store_true",
                        help="Swap which physical PSM each Quest channel drives.")
    parser.add_argument("--cmd-output", choices=["servo_cp", "servo_jp"], default=CMD_OUTPUT,
                        help="servo_cp: send pose, interface runs IK (AMBF). "
                             "servo_jp: run IK in the bridge, send joints (IsaacSim, or "
                             "to validate local IK against AMBF's servo_cp behavior).")
    args = parser.parse_args()

    QUEST_IP = args.quest_ip
    TARGET = args.target
    CMD_OUTPUT = args.cmd_output
    if args.swap:
        SWAP_PSMS = True
    if args.offset_rpy is not None:
        CP_OFFSET_RPY_DEG = tuple(args.offset_rpy)
        R_OFFSET = rpy_deg_to_matrix(*CP_OFFSET_RPY_DEG)

    # The isaac profile has no servo_cp -> IK must run in the bridge. Force it,
    # warning if the user explicitly asked for the impossible combination.
    if not _TOPICS[TARGET]["has_servo_cp"] and CMD_OUTPUT == "servo_cp":
        print(f"[bridge] target '{TARGET}' has no servo_cp; forcing --cmd-output servo_jp.")
        CMD_OUTPUT = "servo_jp"

    rclpy.init()
    node = UdpCrtkBridge()
    try:
        if args.home:
            node.run_home_burst()   # blocking; nothing else runs during homing
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
