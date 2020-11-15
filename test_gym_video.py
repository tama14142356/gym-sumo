import gym
import gym_sumo

kwargs = {
    "step_length": 1,
    "mode": "gui",
    "carnum": 1,
}

env = gym.make("sumo-light-v0", **kwargs)
env = gym.wrappers.Monitor(env, "./video", force=True)

done = False
observation = env.reset()
for i in range(1000):
    # env.render()
    action = env.action_space.sample()
    observation, reward, done, _ = env.step(action)
    if done:
        observation = env.reset()

env.close()
