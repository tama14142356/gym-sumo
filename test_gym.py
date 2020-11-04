import gym
import gym_sumo
# from gym_sumo.envs.sumo_light_env import RIGHT, LEFT
# from gym_sumo.envs.sumo_simple_env import SumoSimpleEnv
# import traci

# default
# kwargs = {
#     'step_length': 0.01,
#     'isgraph': True,
#     'area': 'nishiwaseda',
#     'carnum': 100,
#     'mode': 'gui' (or 'cui'),
#     'simlation_step': 100
# }
carnum = 1
kwargs = {
    'step_length': 1,
    # 'mode': 'cui',
    'carnum': carnum
}
kwargs2 = {
    'step_length': 1,
    # 'mode': 'cui',
    'carnum': carnum,
    'label': 'default2'
}

# env = gym.make('gym_sumo:sumo-extrahard-v0', **kwargs)
# env = gym.make('gym_sumo:sumo-v0', **kwargs)
# env = gym.make('sumo-v0', **kwargs)
env = gym.make('sumo-light-v0', **kwargs)
env2 = gym.make('sumo-light-v0', **kwargs2)
# env = gym.make('sumo-simple-v0', **kwargs)
# err_msg = "%r (%s) invalid" % (env.observation, type(env.observation))
# assert env.observation_space.contains(env.observation), err_msg
# print(env.action_space.n)
observation = env.reset()
observation = env2.reset()
vehID = 'veh0'
for i in range(100):
    action = env.action_space.sample()
    observation, reward, done, _ = env.step(action)
    print(i, reward, done, action, observation)
    env.render()

    if done:
        observation = env.reset()
env.close()
for i in range(100):
    action = env2.action_space.sample()
    observation, reward, done, _ = env2.step(action)
    print(i, reward, done, action)
    env2.render()

    if done:
        observation = env2.reset()
env2.close()
