from gym.envs.registration import register

register(
    id='sumo-v0',
    entry_point='gym_sumo.envs:SumoEnv',
)
register(
    id='sumo-extrahard-v0',
    entry_point='gym_sumo.envs:SumoExtraHardEnv',
)
