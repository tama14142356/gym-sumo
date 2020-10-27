import gym
import gym_sumo
# from gym_sumo.envs.sumo_light_env import RIGHT, LEFT
# from gym_sumo.envs.sumo_simple_env import SumoSimpleEnv
# import traci

# default
# args = {
#     'step_length': 0.01,
#     'isgraph': True,
#     'area': 'nishiwaseda',
#     'carnum': 100,
#     'mode': 'gui' (or 'cui'),
#     'simlation_step': 100
# }
carnum = 1
args = {
    'step_length': 1,
    'mode': 'cui',
    'carnum': carnum
}

# env = gym.make('gym_sumo:sumo-extrahard-v0', **args)
# env = gym.make('gym_sumo:sumo-v0', **args)
# env = gym.make('sumo-v0', **args)
# env = gym.make('sumo-light-v0', **args)
env = gym.make('sumo-simple-v0', **args)
# err_msg = "%r (%s) invalid" % (env.observation, type(env.observation))
# assert env.observation_space.contains(env.observation), err_msg
# print(env.action_space.n)
observation = env.reset()
vehID = 'veh0'
for i in range(10000):
    env.render()
    action = env.action_space.sample()
    observation, reward, done, _ = env.step(action)
    print(i, reward, done, action)

    if done:
        observation = env.reset()
env.close()
