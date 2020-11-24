import gym
import gym_sumo

# default
# kwargs = {
#     "step_length": 0.01,
#     "isgraph": True,
#     "area": 0, ("nishiwaseda")
#     "carnum": 100,
#     "mode": "gui", (or "cui")
#     "simlation_step": 100,
#     "label": "default",
#     "seed": None,
# }
carnum = 1
kwargs = {
    "step_length": 1,
    # "mode": "cui",
    "carnum": carnum,
    "seed": 1,
}
kwargs2 = {
    "step_length": 1,
    # "mode": "cui",
    "carnum": carnum,
    "label": "default2",
    "seed": 2,
}

# env = gym.make("gym_sumo:sumo-extrahard-v0", **kwargs)
# env = gym.make("gym_sumo:sumo-v0", **kwargs)
# env = gym.make("sumo-v0", **kwargs)
env = gym.make("sumo-light-v0", **kwargs)
env2 = gym.make("sumo-light-v0", **kwargs2)
# env = gym.make("sumo-simple-v0", **kwargs)
observation = env.reset()
vehID = "veh0"
for i in range(100):
    action = env.action_space.sample()
    # action = 0
    # if i % 3 == 2:
    #     action = 6
    observation, reward, done, info = env.step(action)
    print(i, reward, done, action, info)
    env.render()

    if done:
        observation = env.reset()
env.close()

observation = env2.reset()
for i in range(100):
    action = env2.action_space.sample()
    # action = 0
    # if i % 3 == 2:
    #     action = 6
    observation, reward, done, info = env2.step(action)
    print(i, reward, done, action, info)
    env2.render()

    if done:
        observation = env2.reset()
env2.close()
