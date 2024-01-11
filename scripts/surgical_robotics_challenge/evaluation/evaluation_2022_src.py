from surgical_robotics_challenge.evaluation import evaluation
from argparse import ArgumentParser

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-t', action='store', dest='team_name', help='Team Name', default='test_team')
    parser.add_argument('-e', action='store', dest='task_evaluation', help='Task to evaluate (1,2 or 3)')

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)
    evaluation.evaluate(parsed_args)