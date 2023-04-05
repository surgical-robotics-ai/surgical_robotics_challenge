import numpy as np
import os

from stable_baselines import HER, DDPG
from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import OrnsteinUhlenbeckActionNoise
from stable_baselines.common.callbacks import CallbackList, CheckpointCallback, EvalCallback
import SRCEnv


def train(training_env: SRCEnv, log_dir='./logs/results'):
    """ Train a HER-DDPG agent on the SRCEnv environment

    Params:
    -------
    training_env: SRCEnv
        The environment used for training
    log_dir: str
        The directory to save the results

    Returns:
    --------
    model: HER
        The trained model
    """

    # Prepare log directory and hyperparameters
    os.makedirs(log_dir, exist_ok=True)
    n_actions = training_env.action_space.shape[0]
    noise_std = 0.2
    action_noise = OrnsteinUhlenbeckActionNoise(
        mean=np.zeros(n_actions),
        sigma=noise_std * np.ones(n_actions)
    )
    model_class = DDPG
    rl_model_kwargs = {
        'actor_lr': 1e-3,
        'critic_lr': 1e-3,
        'action_noise': action_noise,
        'nb_train_steps': 300,
        'nb_rollout_steps': 100,
        'gamma': 0.95,
        'observation_range': (-1.5, 1.5),
        'random_exploration': 0.05,
        'normalize_observations': True,
        'critic_l2_reg': 0.01
    }
    checkpoint_callback = CheckpointCallback(
        save_freq=100000,
        save_path="./ddpg_dvrk_tensorboard/"
    )
    callback = CallbackList([checkpoint_callback])  # , eval_callback])

    # Train the agent
    model = HER('MlpPolicy', training_env, model_class, n_sampled_goal=4,
                goal_selection_strategy='future', verbose=1, buffer_size=1000000, batch_size=128, 
                tensorboard_log='./herddpg_tensorboard', **rl_model_kwargs)
    training_env.reset()
    model.learn(total_timesteps=400000, log_interval=100, callback=callback)
    model.save("her_ddpg")


 def eval(eval_env):
    """ Load a trained model and evaluate it
    
    Params:
    -------
    eval_env: SRCEnv
        The environment used for evaluation, must be wrapped with SRCEnv
    """
    model = HER.load('./her_ddpg', env=eval_env)
    count = 0
    step_num_arr = []
    for _ in range(20):
        number_steps = 0
        obs = eval_env.reset()
        for _ in range(400):
            action, _ = model.predict(obs)
            obs, reward, done, _ = eval_env.step(action)
            number_steps += 1
            # print(obs['achieved_goal'][0:3], obs['desired_goal'][0:3], reward)
            if done:
                step_num_arr.append(number_steps)
                count += 1
                print("----------------It reached terminal state -------------------")
                break
    print(
        "PSM grasped the needle ",
        count,
        " times and the Average step count was ",
        np.average(np.array(step_num_arr))
    )


if __name__ == '__main__':
    ENV_NAME = 'psm/baselink'
    env_kwargs = {
        'action_space_limit': 0.05,
        'goal_position_range': 0.05,
        'position_error_threshold': 0.01,
        'goal_error_margin': 0.0075,
        'joint_limits':
        {
            'lower_limit': np.array([-0.2,
                                    -0.2,
                                    0.1,
                                    -1.5,
                                    -1.5,
                                    -1.5,
                                    -1.5]),
            'upper_limit': np.array([0.2,
                                    0.2,
                                    0.24,
                                    1.5,
                                    1.5,
                                    1.5,
                                    1.5])
        },
        'workspace_limits':
        {
            'lower_limit': np.array([-0.04, -0.03, -0.2]),
            'upper_limit': np.array([0.03, 0.04, -0.091])
        },
        'enable_step_throttling': False,
        'steps_to_print': 10000
    }
    # Training
    print("Creating training_env")
    src_env = SRCEnv(**env_kwargs)
    time.sleep(5)
    src_env.make(ENV_NAME)
    src_env.reset()

    main(training_env=src_env)  #, eval_env=eval_env)
    src_env._client.clean_up()

    # Evaluate learnt policy
    print("Creating eval_env")
    eval_env = SRCEnv(**env_kwargs)
    time.sleep(5)
    eval_env.make(ENV_NAME)
    eval_env.reset()

    load_model(eval_env=eval_env)
    eval_env._client.clean_up()
