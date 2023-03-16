from stable_baselines3.common.callbacks import BaseCallback


class TensorboardCallback(BaseCallback):
    """
    Custom callback for plotting additional values in tensorboard.
    """

    def __init__(self, verbose=0):
        super(TensorboardCallback, self).__init__(verbose)

    def _on_step(self) -> bool:
        # Log scalar value (here a random variable)
        infos = self.locals['infos']
        for idx in range(self.locals['env'].num_envs):
            # only if mean_episode_reward_terms exists in info
            if 'mean_episode_reward_terms' not in infos[idx]:
                continue
            # record log
            if self.locals['dones'][idx]:
                rew_infos = infos[idx]['mean_episode_reward_terms']
                for key in rew_infos.keys():
                    # check if more than 1 env is done, is the first env reward data overwritten?
                    self.logger.record('reward_terms/' + key, rew_infos[key])
        return True