import gym
# import gym_sumo
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
    'mode': 'gui',
    'carnum': carnum
}

# env = gym.make('gym_sumo:sumo-extrahard-v0', **args)
env = gym.make('gym_sumo:sumo-v0', **args)
direction = [0, 6]
print("observation space num: ", env.observation_space.shape[0])
print("action space num: ", env.action_space[0][0].n, env.action_space[0][1].shape[0])

pobs = env.reset()
done = False
while not done:
    act = []
    for i in range(carnum):
        tmp = np.array([random.uniform(0.0, 1.0)], dtype=np.float32)
        tmp2 = 0
        act.append((tmp2, tmp))
    obs, reward, done, _ = env.step(act)
    print(pobs, act, reward, obs, done)
    pobs = obs
env.close()

# act = []
# tmp = np.array([random.uniform(0.0, 1.0)], dtype=np.float32)
# tmp2 = random.randint(0, 1)
# tmp2 = direction[tmp2]
# act.append((tmp2, tmp))

# print(act, tmp, tmp2)