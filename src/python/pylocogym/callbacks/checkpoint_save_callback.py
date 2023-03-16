# save the model, the vecnormalize and export policy

import os
import torch
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback


class CheckpointSaveCallback(BaseCallback):

    def __init__(self,
                 save_freq: int,
                 save_path: str,
                 save_vecnormalize: bool = False,
                 export_policy: bool = False,
                 verbose: int = 0):
        super().__init__(verbose)
        self.save_freq = save_freq
        self.save_path = save_path
        self.save_vecnormalize = save_vecnormalize
        self.export_policy = export_policy

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.save_freq == 0:

            model_path = os.path.join(self.save_path, f"model_{self.num_timesteps}_steps")
            self.model.save(model_path)
            if self.save_vecnormalize:
                vecnormalize_path = os.path.join(self.save_path, f"vecnormalize_{self.num_timesteps}_steps.pkl")
                self.model.get_vec_normalize_env().save(vecnormalize_path)

            if self.verbose > 1:
                print(f"Saving model checkpoint to {model_path}")
                if self.save_vecnormalize:
                    print(f"Saving vecnormalize checkpoint to {vecnormalize_path}")

            if self.export_policy:
                if self.save_vecnormalize:
                    exported_policy = PolicyToExport(self.model.policy.mlp_extractor,
                                                     self.model.policy.action_net,
                                                     self.model.observation_space,
                                                     self.model.action_space,
                                                     self.model.device,
                                                     self.training_env.obs_rms.mean,
                                                     self.training_env.obs_rms.var,
                                                     self.training_env.epsilon,
                                                     self.training_env.clip_obs
                                                     )
                else:
                    exported_policy = PolicyToExport(self.model.policy.mlp_extractor,
                                                     self.model.policy.action_net,
                                                     self.model.observation_space,
                                                     self.model.action_space,
                                                     self.model.device
                                                     )
                observation_size = self.model.observation_space.shape[0]
                test_obs = torch.ones((1, observation_size), device="cpu")
                output_module = torch.jit.script(exported_policy)
                output_module.save(os.path.join(self.save_path, f"exported_policy_{self.num_timesteps}_steps" + ".pt"))
                print("policy converted successfully and saved.")

                if self.verbose > 1:
                    print("____Test_Exported_Model____")
                    with torch.inference_mode():
                        print("model output:")
                        print(self.model.predict(self.training_env.normalize_obs(test_obs.cpu().numpy()), deterministic=True))
                        print("exported policy output:")
                        print(exported_policy.forward(test_obs))
                    print("___________________________")

        return True


class PolicyToExport(torch.nn.Module):
    def __init__(self,
                 extractor,
                 action_net,
                 obs_space,
                 action_space,
                 device,
                 obs_norm_mean=None,
                 obs_norm_var=None,
                 obs_norm_epsilon=None,
                 obs_clip=None,
                 ):
        super().__init__()
        self.device = device
        self.extractor = extractor
        self.action_net = action_net
        self.obs_space = obs_space
        self.action_space = action_space

        if obs_norm_mean is not None:
            self.obs_norm_mean = torch.from_numpy(obs_norm_mean).type(torch.float32).to(self.device)
            self.obs_norm_var = torch.from_numpy(obs_norm_var).type(torch.float32).to(self.device)
            self.obs_norm_epsilon = obs_norm_epsilon
            self.obs_clip = obs_clip
        else:
            self.obs_norm_mean = torch.from_numpy(np.zeros(self.obs_space.shape)).type(torch.float32).to(self.device)
            self.obs_norm_var = torch.from_numpy(np.ones(self.obs_space.shape)).type(torch.float32).to(self.device)
            self.obs_norm_epsilon = 0
            self.obs_clip = 1e8
        self.action_low = torch.from_numpy(action_space.low).type(torch.float32).to(self.device)
        self.action_high = torch.from_numpy(action_space.high).type(torch.float32).to(self.device)
        self.num_obs = self.obs_space.shape

    def forward(self, observation):
        observation = torch.clamp(
            (observation - self.obs_norm_mean) / torch.sqrt(self.obs_norm_var + self.obs_norm_epsilon),
            -self.obs_clip, self.obs_clip)
        action_hidden, _ = self.extractor(observation)
        actions = self.action_net(action_hidden)
        actions = torch.clip(actions, self.action_low, self.action_high)
        return actions

    @torch.jit.export
    def get_num_obs(self):
        return self.num_obs
