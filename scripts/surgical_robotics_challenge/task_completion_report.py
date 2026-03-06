from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty
from ros_abstraction_layer import ral


class TaskCompletionReport:
    def __init__(self, team_name):
        self._team_name = team_name
        self.ral = None
        try:
            self.ral = ral('challenge_report_node')
        except:
            # Already initialized, so ignore
            do_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + self._team_name
        self._task1_pub = self.ral.publisher(prefix + '/task1/', PoseStamped, queue_size=1)
        self._task2_pub = self.ral.publisher(prefix + '/task2/', Bool, queue_size=1)
        self._task3_pub = self.ral.publisher(prefix + '/task3/', Bool, queue_size=1)

    def task_1_report(self, pose):
        print(self._team_name, 'reporting task 1 complete with result: ', pose)
        self._task1_pub.publish(pose)

    def task_2_report(self, complete):
        print(self._team_name, 'reporting task 2 complete with result: ', complete)
        msg = complete if isinstance(complete, Bool) else Bool(data=bool(complete))
        self._task2_pub.publish(msg)

    def task_3_report(self, complete):
        print(self._team_name, 'reporting task 3 complete with result: ', complete)
        msg = complete if isinstance(complete, Bool) else Bool(data=bool(complete))
        self._task3_pub.publish(msg)
