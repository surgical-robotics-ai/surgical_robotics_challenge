from surgical_robotics_challenge.evaluation.evaluation import *
from ambf_client import Client



c = Client('surgical_robotics_challenge_evaluation')
c.connect()
eval_task1 = Task_1_Evaluation(c, 'my_team')
eval_task1.evaluate()
#
# eval_task2 = Task_2_Evaluation(c, 'my_team')
# eval_task2.evaluate()