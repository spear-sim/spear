import unrealai
from unrealai import UnrealEnv, UnrealToGymWrapper
import numpy as np


def loop(env, len, action):
    count = 0
    for i in range(len):
        obs, reward, done, info = env.step(action)
        print(count, obs, reward, done, info)
        count += 1
        if done:
            env.reset()
            print("reset")


# import logging
# logging.getLogger("tornado.general").disabled = True

config = unrealai.utils.load_config()

uenv = UnrealEnv(config["env_config"])
env = UnrealToGymWrapper(uenv)

# env._env.conn.setSynchronousMode(False)

print(
    f"------------------------- is env ready---------------? - {env.conn.is_env_ready()}"
)

env.unreal_env.print_action_spec()

env.unreal_env.print_obs_spec()

print("Observation space :", env.observation_space)

print("Action space: ", env.action_space)

try:
    # execute different actions for diffrent step counts
    loop(env, 100, np.array([1.0, 0.0]))

    loop(env, 100, np.array([0.0, 1.0]))

    env.reset()
finally:
    env.unreal_env.conn.set_synchronous_mode(False)

env.close()
