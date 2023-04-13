import numpy as np
import os, time

# from stable_baselines3 import HER, DDPG, HerReplayBuffer
# from stable_baselines3.ddpg.policies import MlpPolicy
# from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise
# from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback, EvalCallback
import tianshou as ts
import torch, numpy as np, torch.nn as nn
from tianshou.utils.net.common import Net
from torch.utils.tensorboard import SummaryWriter
import gymnasium as gym
from SRCEnv import SRCEnv


def main():
    task = 'SRCEnv'
    log_dir = './logs/results'

    # Hyperparameters
    lr, epoch, batch_size = 1e-3, 10, 128
    train_num, test_num = 1, 1
    gamma, n_step, target_freq = 0.9, 3, 320
    buffer_size = 20000
    eps_train, eps_test = 0.1, 0.05
    step_per_epoch, step_per_collect = 10000, 10
    logger = ts.utils.TensorboardLogger(SummaryWriter(log_dir)) 

    gym.envs.register(id=task, entry_point=SRCEnv)
    train_envs = ts.env.DummyVectorEnv([lambda: gym.make(task) for _ in range(train_num)])
    test_envs = ts.env.DummyVectorEnv([lambda: gym.make(task) for _ in range(test_num)])

    env = gym.make(task)
    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n
    net = Net(state_shape=state_shape, action_shape=action_shape, hidden_sizes=[128, 128, 128])
    optim = torch.optim.Adam(net.parameters(), lr=lr)

    policy = ts.policy.DQNPolicy(net, optim, gamma, n_step, target_update_freq=target_freq)
    # train_collector = ts.data.Collector(policy, train_envs, ts.data.VectorReplayBuffer(buffer_size, train_num), exploration_noise=True)
    train_collector = ts.data.Collector(policy, train_envs, exploration_noise=True)  # because DQN uses epsilon-greedy method
    test_collector = ts.data.Collector(policy, test_envs, exploration_noise=True)  # because DQN uses epsilon-greedy method

    print('Start training...')
    result = ts.trainer.offpolicy_trainer(
        policy, 
        train_collector, 
        test_collector, 
        epoch, 
        step_per_epoch, 
        step_per_collect,
        test_num, 
        batch_size, 
        update_per_step=1 / step_per_collect,
        train_fn=lambda epoch, env_step: policy.set_eps(eps_train),
        test_fn=lambda epoch, env_step: policy.set_eps(eps_test),
        stop_fn=lambda mean_rewards: mean_rewards >= env.spec.reward_threshold,
        logger=logger)
    print(f'Finished training! Use {result["duration"]}')
    torch.save(policy.state_dict(), 'dqn.pth')

    policy.load_state_dict(torch.load('dqn.pth'))
    policy.eval()
    policy.set_eps(eps_test)
    collector = ts.data.Collector(policy, env, exploration_noise=True)
    collector.collect(n_episode=1, render=1 / 35)
    
    

if __name__ == '__main__':
    main()