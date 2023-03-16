from gym.envs.registration import register
from .VanillaEnv import VanillaEnv

register(id="PylocoVanilla-v0", entry_point=VanillaEnv)
