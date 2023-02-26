'''
Base Environment for Surgical Robotics Challenge
'''
import gym
from gym import spaces
import numpy as np
import time
from scripts.surgical_robotics_challenge.psm_arm import PSM
from scripts.surgical_robotics_challenge.ecm_arm import ECM
from scripts.surgical_robotics_challenge.scene import Scene
from scripts.surgical_robotics_challenge.simulation_manager import SimulationManager
from scripts.surgical_robotics_challenge.task_completion_report import TaskCompletionReport

N_DISCRETE_ACTIONS = 0
HEIGHT = 0
WIDTH = 0
N_CHANNELS = 0


def add_break(s):
    time.sleep(s)
    print('-------------')


class CustomEnv(gym.Env):  # TODO: on dVRL parent class is gym.GoalEnv
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):  # TODO: Add more arguments
        # Define action and observation space
        super(CustomEnv, self).__init__()
        # They must be gym.spaces objects    # Example when using discrete actions:
        # Example for using image as input:
        self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
        self.observation_space = spaces.Box(
            low=0, high=255, shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8)

        self.simulation_manager = SimulationManager('my_example_client')
        self.world_handle = self.simulation_manager.get_world_handle()
        self.psm1 = PSM(self.simulation_manager, 'psm1')
        self.psm2 = PSM(self.simulation_manager, 'psm2')
        self.ecm = ECM(self.simulation_manager, 'CameraFrame')
        self.scene = Scene(self.simulation_manager)
        self.task_report = TaskCompletionReport(team_name='my_team_name')

        # Small sleep to let the handles initialize properly
        add_break(0.5)

    def step(self, action):
        # Execute one time step within the environment
        raise NotImplementedError

    def reset(self):
        # Reset the state of the environment to an initial state
        self.world_handle.reset()  # self.world_handle.reset_bodies()
        self.psm2.servo_jp([-0.4, -0.22, 0.139, -1.64, -0.37, -0.11])
        self.psm2.set_jaw_angle(0.8)
        add_break(3.0)

    def render(self, mode='human', close=False):
        '''
        1. run the simulation (using script or smt else) - init()
        2. use the client to update the robots' positions
        3. done. (ambf client will automatically update the positions of the robots in ambf sim and render it)
        '''
        # Render the environment to the screen
        print("PSM1 End-effector pose in Base Frame", self.psm1.measured_cp())
        print("PSM1 Base pose in World Frame", self.psm1.get_T_b_w())
        print("PSM1 Joint state", self.psm1.measured_jp())
        add_break(1.0)
        print("PSM2 End-effector pose in Base Frame", self.psm2.measured_cp())
        print("PSM2 Base pose in World Frame", self.psm2.get_T_b_w())
        print("PSM2 Joint state", self.psm2.measured_jp())
        add_break(1.0)
        # Things are slightly different for ECM as the `measure_cp` returns pose in the world frame
        print("ECM pose in World", self.ecm.measured_cp())
        add_break(1.0)
        # Scene object poses are all w.r.t World
        print("Entry 1 pose in World", self.scene.entry1_measured_cp())
        print("Exit 4 pose in World", self.scene.exit4_measured_cp())

        # TODO: connect render to AMRL's simulation manager
