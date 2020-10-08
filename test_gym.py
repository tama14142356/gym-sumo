import gym
import gym_sumo
import random
import numpy as np

# default
# args = {
#     'step_length': 0.01,
#     'isgraph': True,
#     'area': 'nishiwaseda',
#     'carnum': 100,
#     'mode': 'gui' (or 'cui'),
#     'simlation_step': 100
# }
carnum = 100
args = {
    'step_length': 0.01,
    'mode': 'cui',
    # 'carnum': carnum
}

# env = gym.make('gym_sumo:sumo-extrahard-v0', **args)
# env = gym.make('gym_sumo:sumo-v0', **args)
env = gym.make('sumo-light-v0', **args)
# err_msg = "%r (%s) invalid" % (env.observation, type(env.observation))
# assert env.observation_space.contains(env.observation), err_msg
# print(env.action_space.n)
observation = env.reset()
for i in range(1000):
    env.render()
    action = env.action_space.sample()
    observation, reward, done, _ = env.step(action)
    print(i, reward, done)

    if done:
        observation = env.reset()
env.close()
