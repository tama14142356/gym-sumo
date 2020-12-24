import gym
import gym_sumo

import os
import logging
import datetime
import json
import pathlib


def main(episode, is_msg=True, monitor=True, outdir="./video"):
    mode = "gui" if monitor else "cui"
    kwargs = {
        "step_length": 1,
        "mode": mode,
        "carnum": 1,
        "seed": 5,
        # "debug_view": True,
        "is_random_route": False,  # for debug
        "is_auto": True,
    }

    # env = gym.make("sumo-light-v0", **kwargs)
    env = gym.make("sumo-fix-v0", **kwargs)
    if monitor:
        env = gym.wrappers.Monitor(
            env, outdir, force=True, video_callable=(lambda e: e % 1 == 0)
        )

    log_file_name = os.path.join(outdir, "log.log")
    logging.basicConfig(level=logging.INFO, filename=log_file_name)
    logger = logging.getLogger(__name__)

    arrive_num = 0
    not_arrived_list = []
    for i in range(episode):
        step = 0
        env.reset()
        done = False
        logger.info("episode: %s", i)
        if is_msg:
            print("episode: ", (i + 1), "/", episode)
        while not done:
            action = env.action_space.sample()
            _, reward, done, info = env.step(action)
            logger.info(
                "step: %s action: %s reward: %s speed: %s arrived: %s",
                step,
                action,
                reward,
                info.get("speed", -1.0),
                info.get("is_arrived", False),
            )
            # print("step: ", step, reward, done, action, info["speed"])
            step += 1
        is_arrived = int(reward) == 100 or info.get("is_arrived", False)
        arrive_num += 1 if is_arrived else 0
        msg = "arrived!! " if is_arrived else "not arrived.. "
        if is_msg:
            print(msg)
        logger.info("%sepisode: %s arrived num: %s", msg, i, arrive_num)
        if not is_arrived:
            info = {"finish reward": reward, "steps": step, "episode number": i}
            not_arrived_list.append(info)
    if is_msg:
        print("arrived number: ", arrive_num)
        print("not arrived number: ", len(not_arrived_list))
        print("arrived percentage: ", ((arrive_num / episode) * 100))
    logger.info("not arrived list %s", not_arrived_list)
    logger.info("arrived percentage %s", (arrive_num / episode) * 100)
    env.close()


def prepare_dir(args, basedir):
    cwd_path = os.path.dirname(__file__)
    basedir_abs_path = pathlib.Path(os.path.join(cwd_path, basedir)).resolve()
    timestamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S.%f")
    outdir = os.path.join(basedir_abs_path, timestamp)
    args.outdir = outdir
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "args.txt"), "w") as f:
        if isinstance(args, argparse.Namespace):
            args = vars(args)
        f.write(json.dumps(args))
    return outdir


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument("--outdir", type=str, default="./video/test")
    parser.add_argument("--no-monitor", action="store_true", default=False)
    parser.add_argument("--no-message", action="store_true", default=False)
    parser.add_argument("--episodes", type=int, default=100000)

    args = parser.parse_args()

    outdir = prepare_dir(args, args.outdir)

    main(args.episodes, (not args.no_message), (not args.no_monitor), outdir)
