from gym.envs.registration import register

register(
    id='sumo-v0',
    entry_point='gym_sumo.envs:SumoEnv',
    max_episode_steps=200,
    reward_threshold=25.0,
)
register(
    id='sumo-light-v0',
    entry_point='gym_sumo.envs:SumoLightEnv',
    max_episode_steps=200,
    reward_threshold=25.0,
)
register(
    id='sumo-simple-v0',
    entry_point='gym_sumo.envs:SumoSimpleEnv',
    max_episode_steps=200,
    reward_threshold=25.0,
)
register(
    id='sumo-extrahard-v0',
    entry_point='gym_sumo.envs:SumoExtraHardEnv',
    max_episode_steps=200,
    reward_threshold=25.0,
)
