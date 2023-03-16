import fnmatch
import numpy as np
import os

from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.evaluation import evaluate_policy


class RecordEvalCallback(EvalCallback):
    def __init__(self,
                 eval_env,
                 eval_freq,
                 load_path,
                 deterministic,
                 render,
                 normalize,
                 video_folder,
                 record_video_trigger,
                 video_length,
                 name_prefix
                 ):
        self.load_path = load_path
        self.normalize = normalize
        self.video_folder = video_folder
        self.record_video_trigger = record_video_trigger
        self.video_length = video_length
        self.name_prefix = name_prefix

        super().__init__(eval_env,
                         eval_freq=eval_freq,
                         deterministic=deterministic,
                         render=render)

    def _on_step(self) -> bool:

        continue_training = True

        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:

            # Sync training and eval env if there is VecNormalize
            # if self.model.get_vec_normalize_env() is not None:
            #     try:
            #         sync_envs_normalization(self.training_env, self.eval_env)
            #     except AttributeError as e:
            #         raise AssertionError(
            #             "Training and eval env are not wrapped the same way, "
            #             "see https://stable-baselines3.readthedocs.io/en/master/guide/callbacks.html#evalcallback "
            #             "and warning above."
            #         ) from e

            if self.normalize:
                index = 0
                for file in os.listdir(self.load_path):
                    if fnmatch.fnmatch(file, 'vecnormalize*'):
                        if int(file.split("_")[1]) >= index:
                            index = int(file.split("_")[1])
                vecnormalize_path = self.load_path + "/vecnormalize_" + str(index) + "_steps.pkl"
                if isinstance(self.eval_env, VecNormalize):
                    self.eval_env = VecNormalize.load(vecnormalize_path, self.eval_env.venv)
                else:
                    self.eval_env = VecNormalize.load(vecnormalize_path, self.eval_env)
                self.eval_env.training = False
                self.eval_env.norm_reward = False

            # Reset success rate buffer
            self._is_success_buffer = []

            episode_rewards, episode_lengths = evaluate_policy(
                self.model,
                self.eval_env,
                n_eval_episodes=self.n_eval_episodes,
                render=self.render,
                deterministic=self.deterministic,
                return_episode_rewards=True,
                warn=self.warn,
                callback=self._log_success_callback,
            )

            if self.log_path is not None:
                self.evaluations_timesteps.append(self.num_timesteps)
                self.evaluations_results.append(episode_rewards)
                self.evaluations_length.append(episode_lengths)

                kwargs = {}
                # Save success log if present
                if len(self._is_success_buffer) > 0:
                    self.evaluations_successes.append(self._is_success_buffer)
                    kwargs = dict(successes=self.evaluations_successes)

                np.savez(
                    self.log_path,
                    timesteps=self.evaluations_timesteps,
                    results=self.evaluations_results,
                    ep_lengths=self.evaluations_length,
                    **kwargs,
                )

            mean_reward, std_reward = np.mean(episode_rewards), np.std(episode_rewards)
            mean_ep_length, std_ep_length = np.mean(episode_lengths), np.std(episode_lengths)
            self.last_mean_reward = mean_reward

            if self.verbose > 0:
                print(
                    f"Eval num_timesteps={self.num_timesteps}, " f"episode_reward={mean_reward:.2f} +/- {std_reward:.2f}")
                print(f"Episode length: {mean_ep_length:.2f} +/- {std_ep_length:.2f}")
            # Add to current Logger
            self.logger.record("eval/mean_reward", float(mean_reward))
            self.logger.record("eval/mean_ep_length", mean_ep_length)

            if len(self._is_success_buffer) > 0:
                success_rate = np.mean(self._is_success_buffer)
                if self.verbose > 0:
                    print(f"Success rate: {100 * success_rate:.2f}%")
                self.logger.record("eval/success_rate", success_rate)

            # Dump log so the evaluation results are printed with the correct timestep
            self.logger.record("time/total_timesteps", self.num_timesteps, exclude="tensorboard")
            self.logger.dump(self.num_timesteps)

            if mean_reward > self.best_mean_reward:
                if self.verbose > 0:
                    print("New best mean reward!")
                if self.best_model_save_path is not None:
                    self.model.save(os.path.join(self.best_model_save_path, "best_model"))
                self.best_mean_reward = mean_reward
                # Trigger callback on new best model, if needed
                if self.callback_on_new_best is not None:
                    continue_training = self.callback_on_new_best.on_step()

            # Trigger callback after every evaluation, if needed
            if self.callback is not None:
                continue_training = continue_training and self._on_event()

        return continue_training
