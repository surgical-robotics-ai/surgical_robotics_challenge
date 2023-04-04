'''
Base Environment for Surgical Robotics Challenge
'''
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time


from src.scripts.surgical_robotics_challenge.psm_arm import PSM
from src.scripts.surgical_robotics_challenge.ecm_arm import ECM
from src.scripts.surgical_robotics_challenge.scene import Scene
from src.scripts.surgical_robotics_challenge.simulation_manager import SimulationManager
from src.scripts.surgical_robotics_challenge.task_completion_report import TaskCompletionReport
from src.scripts.surgical_robotics_challenge.utils.task3_init import NeedleInitialization
from src.scripts.surgical_robotics_challenge.evaluation.evaluation import Task_2_Evaluation, Task_2_Evaluation_Report

N_DISCRETE_ACTIONS = 3
HEIGHT = 0
WIDTH = 0
N_CHANNELS = 0


def add_break(s):
    time.sleep(s)
    print('-------------')

class Observation:
    def __init__(self):
        self.state = [0]*13
        self.dist = 0
        self.reward = 0.0
        self.prev_reward = 0.0
        self.cur_reward = 0.0
        self.is_done = False
        self.info = {}
        self.sim_step_no = 0

    def cur_observation(self):
        return np.array(self.state), self.reward, self.is_done, self.info


class CustomEnv(gym.Env):  # TODO: on dVRL parent class is gym.GoalEnv
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        # Define action and observation space
        super(CustomEnv, self).__init__()
        self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
        # limits for psm
        self.psm_action_lims_low = [np.deg2rad(-91.96), np.deg2rad(-60), -0.0, np.deg2rad(-175), np.deg2rad(-90), np.deg2rad(-85)]
        self.psm_action_lims_high = [np.deg2rad(91.96), np.deg2rad(60), 0.240, np.deg2rad(175), np.deg2rad(90), np.deg2rad(85)]

        self.observation_space = spaces.Box(
            low=0, high=255, shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8)
        self.obs = Observation()

        # connect to client using SimulationManager
        self.simulation_manager = SimulationManager('my_example_client')

        # initialize simulation environment
        self.world_handle = self.simulation_manager.get_world_handle()
        self.scene = Scene(self.simulation_manager)
        self.simulation_manager._client.print_summary()
        self.psm1 = PSM(self.simulation_manager, 'psm1')
        self.psm2 = PSM(self.simulation_manager, 'psm2')
        self.ecm = ECM(self.simulation_manager, 'CameraFrame')
        self.needle = NeedleInitialization(self.simulation_manager)

        self.task_report = TaskCompletionReport(team_name='my_team_name')

        # Small sleep to let the handles initialize properly
        add_break(0.5)
        return

    def get_needle_in_world(self):
        T_tINn = self.needle.needle.get_pose() # needle tip position in needle FC
        T_nINw = # TODO Walee # frame transformation from needle to world 
        T_tINw = T_nINw * T_tINn # needle tip position in world FC
        return T_tINw

    def _update_observation(self, action):
        """ Update the observation of the environment

        Parameters
        - action: an action provided by the environment

        Returns

        """
        self.obs.state = self.psm1.measured_jp() + action # jp = jaw position
        print('needle.get_pose()', self.needle.needle.get_pose().p)
        print('psm measured_cp', self.psm1.measured_cp())
        T_tINw = get_needle_in_world()
        self.obs.dist = self.calc_dist(T_tINw.p, self.psm1.measured_cp()) # both in world coordinates
        self.obs.angle = # TODO Walee # angle between needle and psm1
        self.obs.reward = self.reward(action)
        self.obs.info = {}
        self.obs.sim_step_no += 1
        if self.obs.dist < 0.01: # TODO: tune goal threshold, Walee add angle threshold
            self.obs.is_done = True
    

    def step(self, action):
        """ Execute one time step within the environment

        Parameters
        - action: an action provided by the environment

        Returns
        - observation: agent's observation of the current environment after the action

        """
        # action = np.clip(action, self.action_lims_low, self.action_lims_high)
        # TODO: action is already determined to be the best action
        self.action = action

        self.psm1.servo_jp(action)
        self.world_handle.update()
        self._update_observation(action)
        return self.obs.cur_observation()

    def reward(self, action):
        # Return the reward for the action
        reward = 0
        grasp_reward = self.grasp_reward(action)
        reward += grasp_reward
        # task2_eval = Task_2_Evaluation(self.simulation_manager._client, "test")
        # task2_eval.evaluate()
        # success = task2_eval._report.success # record success/completion of Task 2
        return reward

    def calc_dist(self, goal_pose, current_pose):
        dist = np.linalg.norm(goal_pose - current_pose)
        return dist

    def grasp_reward(self, action):
        goal_pose = self.needle.get_pose()
        current_pose = self.psm1.measured_cp() + action
        print('goal_pose: ', goal_pose)
        print('self.psm1.measured_cp(): ', self.psm1.measured_cp())
        print('action: ', action)

        dist = self.calc_dist(goal_pose, current_pose)
        return -dist

    def reset(self):
        # Reset the state of the environment to an initial state
        self.world_handle.reset()  # self.world_handle.reset_bodies()
        self.psm2.servo_jp([-0.4, -0.22, 0.139, -1.64, -0.37, -0.11])
        self.psm2.set_jaw_angle(0.8)
        add_break(3.0)
        action = [0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0]
        return self.step(action)[0]

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
        print("Needle pose in Needle Frame", self.needle.get_tip_to_needle_offset())
        add_break(1.0)
        # Things are slightly different for ECM as the `measure_cp` returns pose in the world frame
        print("ECM pose in World", self.ecm.measured_cp())
        add_break(1.0)

        # Scene object poses are all w.r.t World
        print("Entry 1 pose in World", self.scene.entry1_measured_cp())
        print("Exit 4 pose in World", self.scene.exit4_measured_cp())

        # TODO: connect render to AMRL's simulation manager

    def move_needle(self):
        # First we shall move the PSM to its initial pose using joint commands OR pose command
        self.psm2.servo_jp([-0.4, -0.22, 0.139, -1.64, -0.37, -0.11])
        # Open the Jaws
        self.psm2.set_jaw_angle(0.8)
        # Sleep to achieve the target pose and jaw angle
        time.sleep(1.0)

        psm2_tip = self.simulation_manager.get_obj_handle('psm2/toolyawlink')
        # Sanity sleep
        time.sleep(0.5)
        # This method will automatically start moving the needle to be with the PSM2's jaws
        self.needle.move_to(psm2_tip)
        time.sleep(0.5)
        for i in range(30):
            # Close the jaws to grasp the needle
            # Calling it repeatedly a few times so that the needle is forced
            # between the gripper tips and grasped properly
            self.psm2.set_jaw_angle(0.0)
            time.sleep(0.01)
        time.sleep(0.5)
        # Don't forget to release the needle control loop to move it freely.
        self.needle.release()
        time.sleep(2.0)
        # Open the jaws to let go of the needle from grasp
        self.psm2.set_jaw_angle(0.8)
        time.sleep(2.0)


if __name__ == "__main__":
    env = CustomEnv()
    env.render()
    env.step([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    env.render()
