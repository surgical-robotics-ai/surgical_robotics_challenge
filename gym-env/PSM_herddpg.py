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
    train_num, test_num = 10, 10
    gamma, n_step, target_freq = 0.9, 3, 320
    buffer_size = 20000
    eps_train, eps_test = 0.1, 0.05
    step_per_epoch, step_per_collect = 10000, 10
    logger = ts.utils.TensorboardLogger(SummaryWriter(log_dir)) 

    train_envs = ts.env.DummyVectorEnv([lambda: SRCEnv() for _ in range(train_num)])
    test_envs = ts.env.DummyVectorEnv([lambda: SRCEnv() for _ in range(test_num)])

    gym.envs.register(id=task, entry_point=SRCEnv)
    env = gym.make(task)
    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n
    net = Net(state_shape=state_shape, action_shape=action_shape, hidden_sizes=[128, 128, 128])
    optim = torch.optim.Adam(net.parameters(), lr=lr)

    policy = ts.policy.DQNPolicy(net, optim, gamma, n_step, target_update_freq=target_freq)
    train_collector = ts.data.Collector(policy, train_envs, ts.data.VectorReplayBuffer(buffer_size, train_num), exploration_noise=True)
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
    # ENV_NAME = 'psm/baselink'
    # # env_kwargs = {
    # #     'action_space_limit': 0.05,
    # #     'goal_position_range': 0.05,
    # #     'position_error_threshold': 0.01,
    # #     'goal_error_margin': 0.0075,
    # #     'joint_limits':
    # #     {
    # #         'lower_limit': np.array([-0.2,
    # #                                 -0.2,
    # #                                 0.1,
    # #                                 -1.5,
    # #                                 -1.5,
    # #                                 -1.5,
    # #                                 -1.5]),
    # #         'upper_limit': np.array([0.2,
    # #                                 0.2,
    # #                                 0.24,
    # #                                 1.5,
    # #                                 1.5,
    # #                                 1.5,
    # #                                 1.5])
    # #     },
    # #     'workspace_limits':
    # #     {
    # #         'lower_limit': np.array([-0.04, -0.03, -0.2]),
    # #         'upper_limit': np.array([0.03, 0.04, -0.091])
    # #     },
    # #     'enable_step_throttling': False,
    # #     'steps_to_print': 10000
    # # }
    # # Training
    # print("Creating training_env")
    # src_env = SRCEnv()
    # # src_env = SRCEnv(**env_kwargs)
    # time.sleep(5)
    # # src_env.make(ENV_NAME)
    # src_env.reset()

    # train(training_env=src_env)  #, eval_env=eval_env)
    # src_env._client.clean_up()

    # # Evaluate learnt policy
    # print("Creating eval_env")
    # # eval_env = SRCEnv(**env_kwargs)
    # eval_env = SRCEnv()
    # time.sleep(5)
    # # eval_env.make(ENV_NAME)
    # eval_env.reset()

    # load_model(eval_env=eval_env)
    # eval_env._client.clean_up()
