from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import rospy
import math
from PyKDL import Rotation


class RobotData:
    def __init__(self):
        self.measured_js = JointState()
        self.measured_cp = TransformStamped()


robData = RobotData()


def measured_js_cb(msg):
    robData.measured_js = msg


def measured_cp_cb(msg):
    robData.measured_cp = msg


rospy.init_node("sur_chal_crtk_test")

namespace = "/CRTK/"
arm_name = "psm1"
measured_js_name = namespace + arm_name + "/measured_js"
measured_cp_name = namespace + arm_name + "/measured_cp"
servo_jp_name = namespace + arm_name + "/servo_jp"
servo_cp_name = namespace + arm_name + "/servo_cp"

measured_js_sub = rospy.Subscriber(measured_js_name, JointState, measured_js_cb, queue_size=1)
measured_cp_sub = rospy.Subscriber(measured_cp_name, TransformStamped, measured_cp_cb, queue_size=1)

servo_jp_pub = rospy.Publisher(servo_jp_name, JointState, queue_size=1)
servo_cp_pub = rospy.Publisher(servo_cp_name, TransformStamped, queue_size=1)

rate = rospy.Rate(50)

servo_jp_msg = JointState()
servo_jp_msg.position = [0., 0., 1.0, 0., 0., 0.]

servo_cp_msg = TransformStamped()
servo_cp_msg.transform.translation.z = -1.0
R_7_0 = Rotation.RPY(3.14, 0.0, 1.57079)

servo_cp_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
servo_cp_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
servo_cp_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
servo_cp_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]
while not rospy.is_shutdown():
    # print("measured_js: ", robData.measured_js)
    # print("------------------------------------")
    # print("measured_cp: ", robData.measured_cp.transform)

    # servo_jp_msg.position[0] = 0.2 * math.sin(rospy.Time.now().to_sec())
    # servo_jp_msg.position[1] = 0.2 * math.cos(rospy.Time.now().to_sec())
    # servo_jp_pub.publish(servo_jp_msg)

    servo_cp_msg.transform.translation.x = 0.2 * math.sin(rospy.Time.now().to_sec())
    servo_cp_msg.transform.translation.y = 0.2 * math.cos(rospy.Time.now().to_sec())
    servo_cp_pub.publish(servo_cp_msg)

    rate.sleep()

