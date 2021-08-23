import PyKDL
from PyKDL import Frame, Rotation, Vector
from razer_hydra.msg import Hydra
# from geomagic_control.msg import DeviceFeedback, DeviceButtonEvent
from geometry_msgs.msg import PoseStamped, Twist
import rospy
import time

# Utilities
def kdl_frame_to_pose_msg(kdl_pose):
    #### rotation 0-3 w,x,y,z
    #### translation 0-2 x,y,z
    ps = PoseStamped()
    p = ps.pose
    p.position.x = kdl_pose.p[0]
    p.position.y = kdl_pose.p[1]
    p.position.z = kdl_pose.p[2]

    p.orientation.x = kdl_pose.M.GetQuaternion()[0]
    p.orientation.y = kdl_pose.M.GetQuaternion()[1]
    p.orientation.z = kdl_pose.M.GetQuaternion()[2]
    p.orientation.w = kdl_pose.M.GetQuaternion()[3]

    return ps


def pose_msg_to_kdl_frame(msg_pose):
    pose = msg_pose.pose
    f = Frame()
    f.p[0] = pose.position.x
    f.p[1] = pose.position.y
    f.p[2] = pose.position.z
    f.M = Rotation.Quaternion(pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z,
                              pose.orientation.w)

    return f

def hydra_msg_to_kdl_frame(msg_hydra):
    pose = msg_hydra.paddles[0].transform
    f = Frame()
    f.p[0] = 10*pose.translation.x
    f.p[1] = 10*pose.translation.y
    f.p[2] = 10*pose.translation.z
    f.M = Rotation.Quaternion(pose.rotation.x,
                              pose.rotation.y,
                              pose.rotation.z,
                              pose.rotation.w)

    return f


# Init everything related to Geomagic
class razer_Device:
    # The name should include the full qualified prefix. I.e. '/Geomagic/', or '/omniR_' etc.
    def __init__(self, name = 'hydra_calib'):
        pose_topic_name = name # + 'pose'
        twist_topic_name = name
        # twist_topic_name = name + 'twist'
        # button_topic_name = name + 'button'
        # force_topic_name = name + 'force_feedback'

        self._active = False
        self._scale = 0.0002
        self.pose = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.twist = PyKDL.Twist()
        # This offset is to align the pitch with the view frame
        R_off = Rotation.RPY(0.0, 0, 0)
        self._T_baseoffset = Frame(R_off, Vector(0, 0, 0))
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        self._T_tipoffset = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.clutch_button_pressed = False # Used as Position Engage Clutch
        self.gripper_button_pressed = False # Used as Gripper Open Close Binary Angle
        # self._force = DeviceFeedback()
        # self._force.force.x = 0
        # self._force.force.y = 0
        # self._force.force.z = 0
        # self._force.position.x = 0
        # self._force.position.y = 0
        # self._force.position.z = 0

        self._pose_sub = rospy.Subscriber(pose_topic_name, Hydra, self.pose_cb, queue_size=1)
        self._twist_sub = rospy.Subscriber(twist_topic_name, Hydra, self.twist_cb, queue_size=1)
        # self._button_sub = rospy.Subscriber(button_topic_name, DeviceButtonEvent, self.buttons_cb, queue_size=1)
        # self._force_pub = rospy.Publisher(force_topic_name, DeviceFeedback, queue_size=1)

        self.switch_psm = False

        # self._button_msg_time = rospy.Time.now()
        self._switch_psm_duration = rospy.Duration(0.5)

        # print('Creating Geomagic Device Named: ', name, ' From ROS Topics')
        self._msg_counter = 0

    def set_base_frame(self, frame):
        self._T_baseoffset = frame
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        pass

    def set_tip_frame(self, frame):
        self._T_tipoffset = frame
        pass

    def set_scale(self, scale):
        self._scale = scale

    def get_scale(self):
        return self._scale

    def pose_cb(self, msg):
        # cur_frame = pose_msg_to_kdl_frame(msg)
        cur_frame = hydra_msg_to_kdl_frame(msg)
        cur_frame.p = cur_frame.p * self._scale
        self.pose = self._T_baseoffset_inverse * cur_frame * self._T_tipoffset
        # Mark active as soon as first message comes through
        self._active = True
        pass

    def twist_cb(self, msg):
        twist = PyKDL.Twist()
        state_button_1 = msg.paddles[1].buttons[0]
        state_button_4 = msg.paddles[1].buttons[4]
        state_button_5 = msg.paddles[1].buttons[5]
        if state_button_1:
            para_vel = -1
        else:
            para_vel = 1
        if state_button_5:
            para_ang = 1
        else:
            para_ang = 0
        if state_button_4:
            para_trans = 1
        else:
            para_trans = 0
        twist[0] = msg.paddles[1].joy[0]
        twist[1] = msg.paddles[1].joy[1]
        twist[2] = para_vel*msg.paddles[1].trigger
        twist[3] = 0#para_ang*msg.paddles[1].joy[0]#msg.angular.x
        twist[4] = 0#para_ang*msg.paddles[1].joy[1]#msg.angular.y
        twist[5] = 0#para_ang*para_vel*msg.paddles[1].trigger#msg.angular.z
        self.twist = self._T_baseoffset_inverse * twist
        pass

    def buttons_cb(self, msg):
        self.gripper_button_pressed = msg.white_button
        self.clutch_button_pressed = msg.grey_button

        if self.clutch_button_pressed:
            time_diff = rospy.Time.now() - self._button_msg_time
            if time_diff.to_sec() < self._switch_psm_duration.to_sec():
                print('Allow PSM Switch')
                self.switch_psm = True
            self._button_msg_time = rospy.Time.now()

    def command_force(self, force):
        pass

    def measured_cp(self):
        return self.pose

    def measured_cv(self):
        return self.twist

    def get_jaw_angle(self):
        return 0.0
        # if self.gripper_button_pressed:
        #     return 0.0
        # else:
        #     return 0.3


def test():
    rospy.init_node('test_geomagic')

    d = razer_Device()

    while not rospy.is_shutdown():
        [r, p, y] = d.measured_cp().M.GetRPY()
        f = 180.0 / 3.1404
        r = round(r * f, 2)
        p = round(p * f, 2)
        y = round(y * f, 2)
        print('Roll: ', r, ', Pitch: ', p, ', Yaw: ', y)
        tst = d.measured_cv()
        print(tst.vel)
        time.sleep(0.05)


if __name__ == '__main__':
    test()
