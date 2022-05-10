from surgical_robotics_challenge.utilities import utils
from PyKDL import Frame, Rotation, Vector
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty


class TaskEvaluationSetup:
    def __init__(self, team_name):
        self._team_name = team_name
        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + self._team_name
        self._task1_sub = rospy.Subscriber(prefix + '/task1/', PoseStamped, self.task_1_cb, queue_size=1)
        self._task2_sub = rospy.Subscriber(prefix + '/task2/', Bool, self.task_2_cb, queue_size=1)
        self._task3_sub = rospy.Subscriber(prefix + '/task3/', Bool, self.task_3_cb, queue_size=1)

        self._needle_pose = None
        self._start_time = rospy.Time.now()
        self._completion_time = None

    def task_1_cb(self, msg):
        self._completion_time = rospy.Time.now()
        self._needle_pose = msg

    def task_2_cb(self, msg):
        self._completion_time = rospy.Time.now()

    def task_3_cb(self, msg):
        self._completion_time = rospy.Time.now()


class NeedleKinematics:
    def __int__(self, needle):
        self._neddle = needle
        # Base in Origin
        self._T_bINo = Frame(Rotation.RPY(0., 0., 0.), Vector(-0.102, 0., 0.))
        # Mid in Origin
        self._T_mINo = Frame(Rotation.RPY(0., 0., -1.091), Vector(-0.048, 0.093, 0.))
        # Tip in Origin
        self._T_tINo = Frame(Rotation.RPY(0., 0., -0.585), Vector(0.056, 0.085, 0.))

    def get_tip_pos(self, T_o_in_w):
        T_tINw = T_o_in_w.Inverse() * self._T_tINo


class Task_1_Evaluation:
    def __int__(self, client, team_name):
        self._needle = client.get_obj_handle('Needle')
        self._world = client.get_world_handle()
        self._team_name = team_name
        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + self._team_name
        self._task1_sub = rospy.Subscriber(prefix + '/task1/', PoseStamped, self.task_1_completion_cb, queue_size=1)

    def task_1_completion_cb(self, msg):
        self._completion_time = rospy.Time.now()
        self._needle_pose = msg
