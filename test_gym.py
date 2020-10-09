import gym
import gym_sumo
from gym_sumo.envs.sumo_light_env import RIGHT, LEFT
import traci

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
    # 'mode': 'cui',
    # 'carnum': carnum
}

# env = gym.make('gym_sumo:sumo-extrahard-v0', **args)
# env = gym.make('gym_sumo:sumo-v0', **args)
env = gym.make('sumo-light-v0', **args)
# err_msg = "%r (%s) invalid" % (env.observation, type(env.observation))
# assert env.observation_space.contains(env.observation), err_msg
# print(env.action_space.n)
observation = env.reset()
vehID = 'veh0'
for i in range(1000):
    env.render()
    action = env.action_space.sample()
    action = 0
    signal = traci.vehicle.getSignals(vehID)
    if signal == 0:
        action = RIGHT
    elif signal == 1:
        action = LEFT
    observation, reward, done, _ = env.step(action)
    print(i, reward, done)

    if done:
        observation = env.reset()
env.close()
