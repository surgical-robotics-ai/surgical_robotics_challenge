from PyKDL import Rotation, Frame, Vector
import rospy
from geometry_msgs.msg import TransformStamped, Transform

measured_cp = Frame()
servo_cp = Frame()

ready = False


def transform_to_frame(T):
    F = Frame()
    F.p = Vector(T.translation.x,
                 T.translation.y,
                 T.translation.z)
    F.M = Rotation.Quaternion(T.rotation.x,
                              T.rotation.y,
                              T.rotation.z,
                              T.rotation.w)
    return F


def frame_to_transform(F):
    T = Transform()
    T.translation.x = F.p[0]
    T.translation.y = F.p[1]
    T.translation.z = F.p[2]
    Q = F.M.GetQuaternion()
    T.rotation.x = Q[0]
    T.rotation.y = Q[1]
    T.rotation.z = Q[2]
    T.rotation.w = Q[3]
    return T


def measured_cp_cb(msg):
    global ready, servo_cp, measured_cp
    measured_cp = transform_to_frame(msg.transform)


def servo_cp_cb(msg):
    global ready, servo_cp
    ready = True
    servo_cp = transform_to_frame(msg.transform)


def print_frame(name, F):
    rpy = F.M.GetRPY()
    ro = round(rpy[0], 2)
    pi = round(rpy[1], 2)
    ya = round(rpy[2], 2)
    x = round(F.p[0], 2)
    y = round(F.p[1], 2)
    z = round(F.p[2], 2)
    print(name, ": RPY: ", ro, pi, ya, " | POS: ", x, y, z)


rospy.init_node("cp_test")
measured_cp_sb = rospy.Subscriber("/CRTK/psm1/measured_cp", TransformStamped, measured_cp_cb, queue_size=1)
servo_cp_sb = rospy.Subscriber("/CRTK/psm1/servo_cp", TransformStamped, servo_cp_cb, queue_size=1)

rate = rospy.Rate(60)
while not rospy.is_shutdown():
    print_frame("Measured CP", measured_cp)
    if ready:
        print_frame("Servo    CP", servo_cp)
        rate.sleep()
