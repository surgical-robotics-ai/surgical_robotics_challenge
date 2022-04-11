from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty
import rospy


class ReportPublishers:
    def __init__(self):
        rospy.init_node('challenge_report_node')
        prefix = '/surgical_robotics_challenge/completion_report/'
        self._task1_pub = rospy.Publisher(prefix + '/task1/', PoseStamped, queue_size=1)
        self._task2_pub = rospy.Publisher(prefix + '/task2/', Bool, queue_size=1)
        self._task1_pub = rospy.Publisher(prefix + '/task3/', Bool, queue_size=1)

    def task_1_report(self, pose):
        self._task1_pub.publish(pose)

    def task_2_report(self, complete):
        self._task2_pub.publish(complete)

    def task_3_report(self, complete):
        self._task3_pub.publish(complete)