'''
Base Environment for Surgical Robotics Challenge
'''
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time
import re

from PyKDL import Frame, Rotation, Vector
from gym.spaces.box import Box
from src.scripts.surgical_robotics_challenge.psm_arm import PSM
from src.scripts.surgical_robotics_challenge.ecm_arm import ECM
from src.scripts.surgical_robotics_challenge.scene import Scene
from src.scripts.surgical_robotics_challenge.simulation_manager import SimulationManager
from src.scripts.surgical_robotics_challenge.task_completion_report import TaskCompletionReport
from src.scripts.surgical_robotics_challenge.utils.task3_init import NeedleInitialization
from src.scripts.surgical_robotics_challenge.evaluation.evaluation import Task_2_Evaluation, Task_2_Evaluation_Report
from utils.observation import Observation
from utils.needle_kinematics import NeedleKinematics

N_DISCRETE_ACTIONS = 3
HEIGHT = 0
WIDTH = 0
N_CHANNELS = 0


def add_break(s):
    time.sleep(s)
    print('-------------')

class SRCEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        # Define action and observation space
        super(SRCEnv, self).__init__()
        # Limits for psm
        self.action_lims_low = np.array([np.deg2rad(-91.96), np.deg2rad(-60), -0.0, np.deg2rad(-175), np.deg2rad(-90), np.deg2rad(-85)])
        self.action_lims_high = np.array([np.deg2rad(91.96), np.deg2rad(60), 0.240, np.deg2rad(175), np.deg2rad(90), np.deg2rad(85)])
        self.action_space = spaces.Box(self.action_lims_low, self.action_lims_high)

        self.observation_space = spaces.Box(
            low=0, high=255, shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8)
        self.obs = Observation()

        # Connect to client using SimulationManager
        self.simulation_manager = SimulationManager('src_client')

        # Initialize simulation environment
        self.world_handle = self.simulation_manager.get_world_handle()
        self.scene = Scene(self.simulation_manager)
        self.simulation_manager._client.print_summary()
        self.psm1 = PSM(self.simulation_manager, 'psm1')
        self.psm2 = PSM(self.simulation_manager, 'psm2')
        self.ecm = ECM(self.simulation_manager, 'CameraFrame')
        self.needle = NeedleInitialization(self.simulation_manager) # needle obj
        self._needle_kin = NeedleKinematics() # needle movement and positioning

        # Small sleep to let the handles initialize properly
        add_break(0.5)
        return

    def get_needle_in_world(self):
        """ Get the needle pose in world coordinates """
        return self._needle_kin.get_tip_pose()

    def _update_observation(self, action):
        """ Update the observation of the environment

        Parameters
        - action: an action provided by the environment
        """
        self.obs.state = self.psm1.measured_jp() + action # jp = jaw position

        # Compute current distance and approach angle of psm and needle, both in world coordinates
        self.obs.dist = self.calc_dist(self.get_needle_in_world(), self.psm1.measured_cp())
        self.obs.angle = self.calc_angle(self.psm1) 
        self.obs.reward = self.reward(self.obs)
        self.obs.info = {}
        self.obs.sim_step_no += 1

        # Determine if psm is ready to grasp needle
        if self.obs.dist < 0.01 and self.obs.angle < 0.01:
            self.obs.is_done = True

    def reset(self):
        """ Reset the state of the environment to an initial state """
        self.world_handle.reset()
        self.psm1.servo_jp([-0.4, -0.22, 0.139, -1.64, -0.37, -0.11])
        self.psm1.set_jaw_angle(0.8)
        add_break(3.0)
        action = [0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0]
        return self.step(action)[0]
    

    def step(self, action):
        """ Execute one time step within the environment

        Parameters
        - action: an action provided by the environment

        Returns
        - observation: agent's observation of the current environment after the action

        """
        # Limit PSM action to bounds
        action = np.clip(action, self.action_lims_low, self.action_lims_high)
        self.action = action

        # Move PSM
        self.psm1.servo_jp(action)

        # Update simulation
        self.world_handle.update()
        self._update_observation(action)
        return self.obs.cur_observation()

    def reward(self, obs):
        """ Compute the cumulative reward for the action taken
        
        Parameters
        - obs: an action provided by the environment
        
        Returns
        - reward: the reward for the action
        """
        reward = 0
        grasp_reward = self.grasp_reward(obs)
        # TODO: Uncomment when implemented
        # insert_reward = self.insert_reward(obs)
        # target_reward = self.target_reward(obs)
        reward += grasp_reward
        return reward

    def calc_angle(self, psm):
        """ Compute dot product of needle tip and specific psm tip
        
        Parameters
        - needle: the needle pose in world coordinates
        - psm: the psm pose in world coordinates
        
        Returns
        - angle: the angle between the needle and psm"""
        # TODO double check axes of needle R; between x and y?
        needle_R = str(self.get_needle_in_world().M).replace('[', '').replace(']', '').replace('\n', ' ').replace(';', ' ').replace(',', ' ').split()
        needle_R = np.array([float(i) for i in needle_R]).reshape(3, 3)[0:3, 0:1]
        psm_R = np.array(psm.measured_cp()[0:3, 1:2] * -1)
        return np.dot(np.squeeze(np.asarray(needle_R)), np.squeeze(np.asarray(psm_R)))

    def calc_dist(self, goal_pose, current_pose):
        """ Compute the distance between the goal pose and current pose

        Parameters
        - goal_pose: the goal pose in world coordinates
        - current_pose: the current pose in world coordinates

        Returns
        - dist: the distance between the goal pose and current pose
        """
        # TODO: fix goal_pose is vector, current_pose is float
        if type(goal_pose) == Frame:
            goal_pose_p = np.array([goal_pose.p.x(), goal_pose.p.y(), goal_pose.p.z()])
        elif type(goal_pose) == np.matrix:
            goal_pose_p = goal_pose[0:3, 3]

        if type(current_pose) == Frame:
            current_pose_p = np.array([current_pose.p.x(), current_pose.p.y(), current_pose.p.z()])
        elif type(current_pose) == np.matrix:
            current_pose_p = current_pose[0:3, 3]

        dist = np.linalg.norm(goal_pose_p - current_pose_p)
        return dist

    def grasp_reward(self, obs):
        """ Compute the reward for grasping the neelde in PSM
        
        Parameters
        - obs: the observation of the environment
        
        Returns
        - reward: the reward for grasping the needle in PSM
        """
        return -(obs.dist + obs.angle)

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


if __name__ == "__main__":
    env = SRCEnv()
    env.step([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    env.render()
