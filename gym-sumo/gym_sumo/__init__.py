from gym.envs.registration import register

register(
    id="sumo-light-v0",
    entry_point="gym_sumo.envs:SumoLightEnv",
    max_episode_steps=200,
)
register(
    id="sumo-light-v1",
    entry_point="gym_sumo.envs:SumoLightEnv",
    max_episode_steps=500,
)

register(
    id="sumo-fix-v0",
    entry_point="gym_sumo.envs:SumoFixedEnv",
    max_episode_steps=200,
)
register(
    id="sumo-fix-v1",
    entry_point="gym_sumo.envs:SumoFixedEnv",
    max_episode_steps=500,
)
