from gym.envs.registration import register

register(
    id="sumo-light-v0",
    entry_point="gym_sumo.envs:SumoLightEnv",
    kwargs={"simulation_end": 200, "max_length": 100.0},
    max_episode_steps=200,
)
register(
    id="sumo-light-v1",
    entry_point="gym_sumo.envs:SumoLightEnv",
    kwargs={"simulation_end": 500, "max_length": 250.0},
    max_episode_steps=500,
)

register(
    id="sumo-fix-v0",
    entry_point="gym_sumo.envs:SumoFixedEnv",
    kwargs={"simulation_end": 200, "max_length": 100.0},
    max_episode_steps=200,
)
register(
    id="sumo-fix-v1",
    entry_point="gym_sumo.envs:SumoFixedEnv",
    kwargs={"simulation_end": 500, "max_length": 250.0},
    max_episode_steps=500,
)
