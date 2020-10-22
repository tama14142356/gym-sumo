import gym
import gym_sumo

args = {
    'step_length': 1,
    'mode': 'gui',
}

env = gym.make('sumo-simple-v0', **args)
env = gym.wrappers.Monitor(env, './video', force=True)

done = False
observation = env.reset()
while not done:
    # env.render()
    action = env.action_space.sample()
    observation, reward, done, _ = env.step(action)

env.close()
