import gym
from gym import error, spaces, utils
from gym.utils import seeding

class SumoExtraHardEnv(gym.Env):
    metadata = {'render.modes': ['human']}
