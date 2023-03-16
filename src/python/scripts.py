import pathlib
import datetime
import wandb
import torch.nn
from shutil import copy as sh_copy
from stable_baselines3.common import utils
from stable_baselines3.common.logger import configure
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv, VecNormalize, VecVideoRecorder
from pylocogym.callbacks import RecordEvalCallback, TensorboardCallback, CheckpointSaveCallback
from pylocogym.algorithms import CustomPPO


def manage_save_path(log_dir, name):
    date = datetime.date.today().strftime("%Y-%m-%d-")
    name = date + name
    latest_run_id = utils.get_latest_run_id(log_dir, name)
    save_path = utils.os.path.join(log_dir, f"{name}_{latest_run_id + 1}")
    pathlib.Path(save_path).mkdir(parents=True, exist_ok=True)
    return save_path


def train(params, log_path, dir_name, debug, video_recorder, wandb_log, reward_path=None, config_path='config.json'):
    """create a model and train it"""
    save_path = manage_save_path(log_path, dir_name)

    # =============
    # unpack params
    # =============

    env_id = params['env_id']
    env_params = params['environment_params']
    hyp_params = params['train_hyp_params']
    model_params = params['model_params']
    reward_params = params['reward_params']

    n_envs = hyp_params['num_envs'] if (not debug) else 1
    max_episode_steps = hyp_params.get('max_episode_steps', 500)
    max_evaluation_steps = hyp_params.get('max_evaluation_steps', 500)
    seed = hyp_params.get("seed", 313)

    if reward_path is not None:
        reward_params["reward_file_path"] = reward_path  # add reward path to reward params

    # =============
    # weights and biases
    # =============

    if wandb_log:
        wandb.init(
            project="pyloco",
            name=save_path.split("/")[-1],
            config=params,
            sync_tensorboard=True,
            monitor_gym=video_recorder,
            dir=log_path
        )
        wandb.save(config_path)
        if reward_path is not None:
            wandb.save(reward_path)

    # =============
    # create vectorized environment for training
    # =============

    env_kwargs = {"max_episode_steps": max_episode_steps, "env_params": env_params, "reward_params": reward_params}
    env = make_vec_env(
        env_id,
        n_envs=n_envs,
        seed=seed,
        env_kwargs=env_kwargs,
        vec_env_cls=DummyVecEnv if debug else SubprocVecEnv,
    )

    if hyp_params["normalize_observation"] or hyp_params["normalize_reward"]:
        env = VecNormalize(
            env, training=True,
            norm_obs=hyp_params["normalize_observation"],
            norm_reward=hyp_params["normalize_reward"]
        )

    # =============
    # create a single environment for evaluation (and video recording)
    # =============

    eval_env = make_vec_env(
        env_id,
        n_envs=1,
        seed=seed,
        env_kwargs=env_kwargs,
        vec_env_cls=DummyVecEnv,
    )

    eval_env = VecVideoRecorder(
        eval_env,
        video_folder=save_path,
        record_video_trigger=lambda x: x == 0,
        video_length=max_evaluation_steps,
        name_prefix=f"eval-video-{env_id}"
    )

    # =============
    # call backs
    # =============

    # check point: store a trained model
    checkpoint_save_callback = CheckpointSaveCallback(
        save_freq=hyp_params['save_freq'],
        save_path=save_path,
        save_vecnormalize=hyp_params["normalize_observation"] or hyp_params["normalize_reward"],
        export_policy=True,
    )

    # tensorboard: log reward terms
    tensorboard_callback = TensorboardCallback()

    # evaluation w or w/o loaded vecnormalize wrapper: record video
    video_callback = RecordEvalCallback(
        eval_env,
        eval_freq=hyp_params['save_freq'],
        load_path=save_path,
        deterministic=True,
        render=False,
        normalize=(hyp_params["normalize_observation"] or hyp_params["normalize_reward"]),
        video_folder=save_path,
        record_video_trigger=lambda x: x == 0,
        video_length=max_evaluation_steps,
        name_prefix=f"eval-video-{env_id}"
    )

    callbacks = [checkpoint_save_callback]
    if not debug:
        callbacks.append(tensorboard_callback)
    if video_recorder:
        callbacks.append(video_callback)

    # =============
    # create model
    # =============

    activation_fn = torch.nn.Tanh
    if model_params['activation_fn'] == 'ReLu':
        activation_fn = torch.nn.ReLU
    elif model_params['activation_fn'] == 'TanH':
        activation_fn = torch.nn.Tanh
    elif model_params['activation_fn'] == 'Swish':
        activation_fn = torch.nn.SiLU
    elif model_params['activation_fn'] == 'ELU':
        activation_fn = torch.nn.ELU

    policy_kwargs = dict(
        activation_fn=activation_fn,
        log_std_init=model_params['log_std_init'],
        net_arch=[dict(
            pi=model_params['network_architecture']['pi'],
            vf=model_params['network_architecture']['vf']
        )]
    )

    model = CustomPPO(
        'MlpPolicy', env,
        learning_rate=hyp_params['learning_rate'],
        batch_size=hyp_params['batch_size'],
        n_epochs=hyp_params['n_epochs'],
        n_steps=hyp_params['n_steps'],
        vf_coef=hyp_params['vf_coef'],
        policy_kwargs=policy_kwargs,
        seed=seed
    )

    logging_format = ["stdout"]
    if not debug:
        logging_format.append("tensorboard")
    my_logger = configure(save_path, logging_format)
    model.set_logger(my_logger)

    # =============
    # save config file and reward file
    # =============

    sh_copy(config_path, utils.os.path.join(save_path, "config.json"))
    if reward_path is not None:
        sh_copy(reward_path, utils.os.path.join(save_path, "reward.py"))

    # =============
    # start training
    # =============

    model.learn(total_timesteps=hyp_params['time_steps'], callback=callbacks)
    model.save(save_path + "/model")

    del model
