import argparse
import os
import psutil
from ray import tune
from yacs.config import CfgNode
import spear

from envs import PhysicalObservationEnv, VisualObservationEnv
from model import get_model_config_conv, get_model_config_fc
from env_base import SimpleEnv

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--observation_mode", default="simple")
    parser.add_argument("--resume", action="store_true", default=False)
    parser.add_argument("--run_name")
    args = parser.parse_args()

    # RLlib overwrites this environment variable, so we copy it into env_config before invoking RLlib.
    # See https://docs.ray.io/en/master/ray-core/using-ray-with-gpus.html

    config = spear.get_config(
        user_config_files=[
            os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml")),
            os.path.realpath(os.path.join(common_dir, "default_config.common.yaml"))])

    spear.configure_system(config)
    if args.observation_mode == "simple":
        env = SimpleEnv
        model_config = get_model_config_fc()
    elif args.observation_mode == "physical":
        env = PhysicalObservationEnv
        model_config = get_model_config_fc()
    elif args.observation_mode == "visual":
        env = PhysicalObservationEnv
        model_config = get_model_config_conv(480, 640)
    else:
        assert False

    env_config = {"config": config}

    ray_config = {
        "env": env,
        "num_workers": 1,
        "num_gpus":0,
        "env_config": env_config,
        "model": model_config,
        "framework": "torch",
        "disable_env_checking": True,
        "simple_optimizer":True,
        "log_level":"INFO"
    }

    experiment_analysis = tune.run(
        "PPO",
        stop={"episode_reward_mean": 90.0},
        config=ray_config,
        checkpoint_freq=1,
        checkpoint_at_end=True,
        log_to_file=True,
        resume=args.resume,
        name=args.run_name
    )

    assert experiment_analysis.get_last_checkpoint() is not None

    print("\n\n\nLast checkpoint: " + experiment_analysis.get_last_checkpoint() + "\n\n\n")
