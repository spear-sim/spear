import argparse
import os
import psutil
from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from yacs.config import CfgNode

from unrealai.config import get_config

from envs import PhysicalObservationEnv, VisualObservationEnv, MixedObservationEnv, PhysicalOnlyFromMixedObservationEnv, VisualOnlyFromMixedObservationEnv, DebugEnv
from model import get_model_config_conv, get_model_config_fc

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_executable", required=True)
    parser.add_argument("--default_config_file", required=True)
    parser.add_argument("--user_config_file", required=True)
    parser.add_argument("--num_workers", required=True, type=int)
    parser.add_argument("--observation_mode", required=True)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--run_name")
    args = parser.parse_args()

    if args.resume:
        assert args.run_name is not None

    config = get_config(config_files=[args.default_config_file, args.user_config_file])

    config.defrost()

    if args.resume:
        experiment_analysis = tune.ExperimentAnalysis(os.path.join(config.TRAIN.RAY_RESULTS_DIR, args.run_name))
        assert experiment_analysis.get_last_checkpoint() is not None
        print("\n\n\nResuming " + args.run_name + " at checkpoint: " + experiment_analysis.get_last_checkpoint() + "\n\n\n")

    if "TRAIN" not in config:
        config.TRAIN = CfgNode(new_allowed=True)

    # RLlib overwrites this environment variable, so we copy it into env_config before invoking RLlib.
    # See https://docs.ray.io/en/master/ray-core/using-ray-with-gpus.html
    if "CUDA_VISIBLE_DEVICES" in os.environ:
        config.TRAIN.CUDA_VISIBLE_DEVICES = os.environ["CUDA_VISIBLE_DEVICES"]
    else:
        config.TRAIN.CUDA_VISIBLE_DEVICES = ""

    config.UNREALAI.STANDALONE_EXECUTABLE = args.unreal_executable

    config.freeze()

    # If the standalone executable is already running, then kill the existing instance because we won't be able to establish a new connection to it
    for pid in psutil.pids():
        try:
            process = psutil.Process(pid)
            if process.exe().startswith(os.path.splitext(os.path.abspath(config.UNREALAI.STANDALONE_EXECUTABLE))[0]):
                print("\n\n\nWARNING: Killing existing instance: " + process.exe() + "\n\n\n")
                process.terminate()
                process.kill()
        except psutil.AccessDenied:
            pass
        except psutil.NoSuchProcess:
            pass

    if args.observation_mode == "physical":
        env = PhysicalObservationEnv
        model_config = get_model_config_fc()

    elif args.observation_mode == "visual":
        env = VisualObservationEnv
        model_config = get_model_config_conv(config.TRAIN.CONV_IMAGE_HEIGHT, config.TRAIN.CONV_IMAGE_WIDTH)

    elif args.observation_mode == "mixed":
        env = MixedObservationEnv
        model_config = get_model_config_conv(config.TRAIN.CONV_IMAGE_HEIGHT, config.TRAIN.CONV_IMAGE_WIDTH)

    elif args.observation_mode == "physical_only_from_mixed":
        env = PhysicalOnlyFromMixedObservationEnv
        model_config = get_model_config_fc()

    elif args.observation_mode == "visual_only_from_mixed":
        env = VisualOnlyFromMixedObservationEnv
        model_config = get_model_config_conv(config.TRAIN.CONV_IMAGE_HEIGHT, config.TRAIN.CONV_IMAGE_WIDTH)

    elif args.observation_mode == "debug":
        env = DebugEnv
        model_config = get_model_config_fc()

    else:
        assert False

    env_config = {"config": config}

    ray_config = {
        "env": env,
        "num_workers": args.num_workers,
        "env_config": env_config,
        "model": model_config,
    }

    print("ray_config =")
    print(ray_config)

    experiment_analysis = tune.run(
        PPOTrainer,
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
