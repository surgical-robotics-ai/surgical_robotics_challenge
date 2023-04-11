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


# def train(training_env: SRCEnv, log_dir='./logs/results'):
#     """ Train a HER-DDPG agent on the SRCEnv environment

#     Params:
#     -------
#     training_env: SRCEnv
#         The environment used for training
#     log_dir: str
#         The directory to save the results

#     Returns:
#     --------
#     model: HER
#         The trained model
#     """

#     # Prepare log directory and hyperparameters
#     os.makedirs(log_dir, exist_ok=True)
#     n_actions = 3
#     # n_actions = training_env.action_space.shape[0]
#     noise_std = 0.2
#     action_noise = OrnsteinUhlenbeckActionNoise(
#         mean=np.zeros(n_actions),
#         sigma=noise_std * np.ones(n_actions)
#     )
#     model_class = DDPG
#     rl_model_kwargs = {
#         'actor_lr': 1e-3,
#         'critic_lr': 1e-3,
#         'action_noise': action_noise,
#         'nb_train_steps': 300,
#         'nb_rollout_steps': 100,
#         'gamma': 0.95,
#         'observation_range': (-1.5, 1.5),
#         'random_exploration': 0.05,
#         'normalize_observations': True,
#         'critic_l2_reg': 0.01
#     }
#     checkpoint_callback = CheckpointCallback(
#         save_freq=100000,
#         save_path="./ddpg_dvrk_tensorboard/"
#     )
#     callback = CallbackList([checkpoint_callback])  # , eval_callback])

#     # Train the agent
#     # model = HER('MlpPolicy', training_env, model_class, n_sampled_goal=4,
#     #             goal_selection_strategy='future', verbose=1, buffer_size=1000000, batch_size=128, 
#     #             tensorboard_log='./herddpg_tensorboard', **rl_model_kwargs)
#     model = model_class(
#         'MlpPolicy', 
#         training_env, 
#         replay_buffer_class=HerReplayBuffer, 
#         # Parameters for HER
#         replay_buffer_kwargs=dict(
#             n_sampled_goal=4,
#             goal_selection_strategy='future',
#             buffer_size=1000000,
#             batch_size=128,
#             tensorboard_log='./herddpg_tensorboard'
#         ),
#         verbose=1
#     )
#     training_env.reset()
#     model.learn(total_timesteps=400000, log_interval=100, callback=callback)
#     model.save("her_ddpg")


# def eval(eval_env):
#     """ Load a trained model and evaluate it
    
#     Params:
#     -------
#     eval_env: SRCEnv
#         The environment used for evaluation, must be wrapped with SRCEnv
#     """
#     model = HER.load('./her_ddpg', env=eval_env)
#     count = 0
#     step_num_arr = []
#     for _ in range(20):
#         number_steps = 0
#         obs = eval_env.reset()
#         for _ in range(400):
#             action, _ = model.predict(obs)
#             obs, reward, done, _ = eval_env.step(action)
#             number_steps += 1
#             # print(obs['achieved_goal'][0:3], obs['desired_goal'][0:3], reward)
#             if done:
#                 step_num_arr.append(number_steps)
#                 count += 1
#                 print("----------------It reached terminal state -------------------")
#                 break
#     print(
#         "PSM grasped the needle ",
#         count,
#         " times and the Average step count was ",
#         np.average(np.array(step_num_arr))
#     )


def main():
    task = 'SRCEnv'
    log_dir = './logs/results'

    # Hyperparameters
    lr, epoch, batch_size = 1e-3, 10, 128
    train_num, test_num = 10, 100
    gamma, n_step, target_freq = 0.9, 3, 320
    buffer_size = 20000
    eps_train, eps_test = 0.1, 0.05
    step_per_epoch, step_per_collect = 10000, 10
    logger = ts.utils.TensorboardLogger(SummaryWriter(log_dir)) 

    train_envs = SRCEnv()
    test_envs = SRCEnv()

    # env = gym.make(task)
    state_shape = train_envs.observation_space.shape or train_envs.observation_space.n
    action_shape = train_envs.action_space.shape or train_envs.action_space.n
    net = Net(state_shape=state_shape, action_shape=action_shape, hidden_sizes=[128, 128, 128])
    optim = torch.optim.Adam(net.parameters(), lr=lr)

    policy = ts.policy.DQNPolicy(net, optim, gamma, n_step, target_update_freq=target_freq)
    train_collector = ts.data.Collector(policy, train_envs, ts.data.VectorReplayBuffer(buffer_size, train_num), exploration_noise=True)
    test_collector = ts.data.Collector(policy, test_envs, exploration_noise=True)  # because DQN uses epsilon-greedy method

    result = ts.trainer.offpolicy_trainer(
    policy, train_collector, test_collector, epoch, step_per_epoch, step_per_collect,
    test_num, batch_size, update_per_step=1 / step_per_collect,
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
