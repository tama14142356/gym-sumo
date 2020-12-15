import gym
import gym_sumo

import gym_sumo.envs.sumo_base_env as base

STRAIGHT = base.STRAIGHT
LEFT = base.LEFT
RIGHT = base.RIGHT
PAR_LEFT = base.PAR_LEFT
PAR_RIGHT = base.PAR_RIGHT
UTURN = base.UTURN
direction = base.DIRECTION

# seed=3: debug用
# seed=1: turn タイミング調査済み
# seed=2: ルート短め
# seed=5: 目的地に到達できるように調整済み

kwargs = {
    "step_length": 1,
    "mode": "gui",
    "carnum": 1,
    "seed": 5,
    # "debug_view": True,
}

env = gym.make("sumo-light-v0", **kwargs)
env = gym.wrappers.Monitor(
    env, "./video", force=True, video_callable=(lambda e: e % 1 == 0)
)
action_seed_1 = {18: STRAIGHT, 19: LEFT, 20: LEFT, 22: LEFT}
action_seed_2 = {9: UTURN, 10: UTURN}
for i in range(11, 19):
    action_seed_2[i] = 9
for i in range(19, 29):
    action_seed_2[i] = STRAIGHT
for i in range(29, 44):
    action_seed_2[i] = LEFT
action_seed_4 = {14: STRAIGHT, 17: STRAIGHT, 18: RIGHT}
action_seed_5 = {6: UTURN, 7: UTURN}
for i in range(8, 76):
    action_seed_5[i] = STRAIGHT
for i in range(76, 81):
    action_seed_5[i] = RIGHT
cur_sm_time = 0.0
episode = 2
for i in range(episode):
    step = 0
    observation = env.reset()
    done = False
    print(i)
    while not done:
        # env.render()
        action = env.action_space.sample()
        # action = 0
        # if i % 3 == 2:
        #     action = 6
        # if kwargs["seed"] == 1:
        #     action = action_seed_1.get(int(cur_sm_time), action)
        # if kwargs["seed"] == 2:
        #     action = action_seed_2.get(i, action)
        # if kwargs["seed"] == 4:
        #     action = action_seed_1.get(i, action)
        # if kwargs["seed"] == 5:
        #     action = action_seed_5.get(i, action)
        # turn_direction = observation[4:10]
        # directions = []
        # for i, turn in enumerate(turn_direction):
        #     if turn == 1:
        #         directions.append(i)
        # if len(directions) > 0:
        #     action_candidate = env.np_random.choice(np.array(directions))
        #     if action_candidate != STRAIGHT:
        #         action = action_candidate
        observation, reward, done, info = env.step(action)
        print(step, reward, done, action, info["speed"])
        step += 1
        # print(i, reward, done, action, info)
        cur_sm_time = info.get("cur_sm_step", 0.0)

env.close()
