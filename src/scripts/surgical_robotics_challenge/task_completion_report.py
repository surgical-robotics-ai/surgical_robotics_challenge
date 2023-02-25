from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty
import rospy


class TaskCompletionReport:
    def __init__(self, team_name):
        self._team_name = team_name
        try:
            rospy.init_node('challenge_report_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + self._team_name
        self._task1_pub = rospy.Publisher(prefix + '/task1/', PoseStamped, queue_size=1)
        self._task2_pub = rospy.Publisher(prefix + '/task2/', Bool, queue_size=1)
        self._task3_pub = rospy.Publisher(prefix + '/task3/', Bool, queue_size=1)

    def task_1_report(self, pose):
        print(self._team_name, 'reporting task 1 complete with result: ', pose)
        self._task1_pub.publish(pose)

    def task_2_report(self, complete):
        print(self._team_name, 'reporting task 2 complete with result: ', complete)
        self._task2_pub.publish(complete)

    def task_3_report(self, complete):
        print(self._team_name, 'reporting task 3 complete with result: ', complete)
        self._task3_pub.publish(complete)
