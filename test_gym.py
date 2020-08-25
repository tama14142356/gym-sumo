import gym
# import gym_sumo
import random

# default
# args = {
#     'step_length': 0.01,
#     'isgraph': True,
#     'area': 'nishiwaseda',
#     'carnum': 100,
#     'mode': 'gui' or 'cui'
# }
args = {
    'step_length': 1,
}
env = gym.make('gym_sumo:sumo-v0', **args)

for i in range(10):
    action = []
    direction = [0, 6]
    for i in range(100):
        tmp = random.uniform(-1.0, 1.0)
        tmp2 = random.randint(0, 1)
        tmp2 = direction[tmp2]
        action.append((tmp2, tmp))

    env.step(action)
    env.render()
env.close()
