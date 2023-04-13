import numpy as np

class Observation:
    def __init__(self):
        self.state = [0]*6
        self.dist = 0
        self.reward = 0.0
        self.prev_reward = 0.0
        self.cur_reward = 0.0
        self.is_done = False
        self.is_truncated = False
        self.info = {}
        self.sim_step_no = 0

    def cur_observation(self):
        return np.array(self.state), np.array(self.reward), np.array(self.is_done), np.array(self.is_truncated), self.info