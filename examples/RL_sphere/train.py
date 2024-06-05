import argparse
import os
import psutil
from ray import tune
import spear

from envs import PhysicalObservationEnv, VisualObservationEnv
from model import get_model_config_conv, get_model_config_fc
from env_base import SimpleEnv
from env_base_simple import SimpleEnv2

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--observation_mode", default="simplePointNav")
    parser.add_argument("--resume", action="store_true", default=False)
    parser.add_argument("--check_point", default=r"C:\Users\admin\ray_results")
    parser.add_argument("--run_name", default="PPO_simplePointNav")
    args = parser.parse_args()

    # RLlib overwrites this environment variable, so we copy it into env_config before invoking RLlib.
    # See https://docs.ray.io/en/master/ray-core/using-ray-with-gpus.html

    if args.resume:
        experiment_analysis = tune.ExperimentAnalysis(os.path.join(args.check_point, args.run_name))
        assert experiment_analysis.get_last_checkpoint() is not None
        print("Resuming ", args.run_name, " at checkpoint: ", experiment_analysis.get_last_checkpoint(), "\n\n\n")

    config = spear.get_config(
        user_config_files=[
            os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml")),
            os.path.realpath(os.path.join(common_dir, "default_config.common.yaml"))])

    spear.configure_system(config)
    if args.observation_mode == "simplePointNav":
        env = SimpleEnv
        # model_config = get_model_config_fc()
    elif args.observation_mode == "dummyPointNav":
        env = SimpleEnv2
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
        # "num_gpus":0,
        "env_config": env_config,
        # "model": model_config,
        "framework": "torch",
        "disable_env_checking": True,
        "log_level": "INFO",
    }

    experiment_analysis = tune.run(
        "PPO",
        stop={"episode_reward_mean": 800.0},
        config=ray_config,
        checkpoint_freq=10,
        checkpoint_at_end=True,
        log_to_file=True,
        resume=args.resume,
        name=args.run_name
    )

    assert experiment_analysis.get_last_checkpoint() is not None
    print("experiment_analysis.get_last_checkpoint()", experiment_analysis.get_last_checkpoint())

    print("\n\n\nLast checkpoint: " + str(experiment_analysis.get_last_checkpoint()) + "\n\n\n")
