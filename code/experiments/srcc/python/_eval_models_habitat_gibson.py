import argparse
from collections import defaultdict
import os
import pickle

import numpy as np
import torch

import gym.spaces

from habitat.config import Config as CN
from habitat.utils.visualizations.utils import images_to_video, observations_to_image

from habitat_baselines.common.baseline_registry import baseline_registry
from habitat_baselines.common.environments import get_env_class, NavRLEnv
from habitat_baselines.config.default import get_config
from habitat_baselines.utils.common import batch_obs, generate_video
from habitat_baselines.utils.env_utils import construct_envs

from my_habitat_baselines.resnet_policy import PointNavResNetPolicy



srcc_dir    = "/Users/mroberts/code/github/interiorsim/code/experiments/srcc"
habitat_dir = "/Users/mroberts/code/github/habitat-lab"

sim_name                  = "habitat_gibson_val"
sim_eval_noise_multiplier = 0.0

depth_prediction_model_path = "/Users/mroberts/code/github/interiorsim/code/experiments/srcc/models/mobilenet-nnconv5dw-skipadd-pruned.pth.tar"
max_depth = 10.0

print()
print()
print()
print()
print()

#
# specify args (outer)
#

parser = argparse.ArgumentParser()
parser.add_argument("--sim_eval_mode", type=str, required=True)
parser.add_argument("--model_name", type=str, required=True)
parser.add_argument("--model_type", type=str, required=True)
parser.add_argument("--model_path", type=str, required=True)
args = parser.parse_args()

print(args)
print()

sim_eval_mode = args.sim_eval_mode
model_name    = args.model_name
model_type    = args.model_type
model_path    = args.model_path

print(sim_name)
print(sim_eval_noise_multiplier)
print(sim_eval_mode)
print(model_name)
print(model_type)
print(model_path)
print()



#
# specify args (inner)
#

arg_string = ""
arg_string += "--model-path %s" % model_path

if model_type == "rgb":
    arg_string += " --sensors RGB_SENSOR"
    arg_string += " --normalize-visual-inputs 1"
elif model_type == "depth":
    arg_string += " --sensors DEPTH_SENSOR"
    arg_string += " --normalize-visual-inputs 0"
elif model_type == "predicted_depth":
    arg_string += " --sensors RGB_SENSOR"
    arg_string += " --normalize-visual-inputs 0"
else:
    assert False

arg_string += \
"""
--hidden-size 512
--backbone resnet50
--num-recurrent-layers 2
TEST_EPISODE_COUNT 5
TASK_CONFIG.SIMULATOR.NOISE_MODEL.CONTROLLER Proportional
TASK_CONFIG.SIMULATOR.RGB_SENSOR.HFOV 45
TASK_CONFIG.SIMULATOR.DEPTH_SENSOR.HFOV 45
TASK_CONFIG.SIMULATOR.TURN_ANGLE 30
TASK_CONFIG.SIMULATOR.AGENT_0.RADIUS 0.20
TASK_CONFIG.DATASET.DATA_PATH obstacle_1/{split}/{split}.json.gz
TASK_CONFIG.DATASET.SPLIT minival
TASK_CONFIG.ENVIRONMENT.GENERATE_ON_FLY False
TASK_CONFIG.SIMULATOR.RGB_SENSOR.POSITION [0,0.6096,0]
TASK_CONFIG.SIMULATOR.DEPTH_SENSOR.POSITION [0,0.6096,0]
VIDEO_OPTION ['disk']
TASK_CONFIG.TASK.TOP_DOWN_MAP.MAP_RESOLUTION 5000
"""

arg_string += "TASK_CONFIG.SIMULATOR.NOISE_MODEL.NOISE_MULTIPLIER " + str(sim_eval_noise_multiplier)

parser = argparse.ArgumentParser()
parser.add_argument("--model-path", type=str, required=True)
parser.add_argument("--sensors", type=str, required=True)
parser.add_argument("--hidden-size", type=int, required=True)
parser.add_argument(
    "--normalize-visual-inputs", type=int, required=True, choices=[0, 1]
)
parser.add_argument(
    "--backbone",
    type=str,
    required=True,
    choices=["resnet50", "se_resneXt50"],
)
parser.add_argument("--num-recurrent-layers", type=int, required=True)
parser.add_argument(
    "opts",
    default=None,
    nargs=argparse.REMAINDER,
    help="Modify config options from command line",
)
args = parser.parse_args(arg_string.split())

print(args)
print()



#
# load and customize config
#

os.chdir(habitat_dir)

config = get_config(
    "habitat_baselines/config/pointnav/ppo_pointnav.yaml"
)

config.defrost()
config.TASK_CONFIG.SIMULATOR.NOISE_MODEL = CN()
config.TASK_CONFIG.SIMULATOR.NOISE_MODEL.CONTROLLER = None
config.TASK_CONFIG.SIMULATOR.NOISE_MODEL.NOISE_MULTIPLIER = None
config.TASK_CONFIG.SIMULATOR.RGB_SENSOR.HFOV = None
config.TASK_CONFIG.SIMULATOR.DEPTH_SENSOR.HFOV = None
config.TASK_CONFIG.ENVIRONMENT.GENERATE_ON_FLY = None
config.freeze()

config.merge_from_list(args.opts)

# config.defrost()
# config.TASK_CONFIG.SIMULATOR.ACTION_SPACE_CONFIG = "pyrobotnoisy"
# config.freeze()

# config.defrost()
# config.TASK_CONFIG.SIMULATOR.NOISE_MODEL.ROBOT = "LoCoBot"
# config.TASK_CONFIG.SIMULATOR.NOISE_MODEL.CONTROLLER = "ILQR"    # our pre-trained model lists "proportional" in the filename, so don't change to ILQR 
# config.TASK_CONFIG.SIMULATOR.NOISE_MODEL.NOISE_MULTIPLIER = 1.0 # our pre-trained model lists "0.5" in the filename, so don't change to 1.0
# config.freeze()

config.defrost()
config.TASK_CONFIG.ENVIRONMENT.ITERATOR_OPTIONS.SHUFFLE = False
config.freeze()

if model_type == "rgb":
    config.defrost()
    config.TASK_CONFIG.SIMULATOR.AGENT_0.SENSORS = ["RGB_SENSOR"]
    config.freeze()
elif model_type == "depth":
    config.defrost()
    config.TASK_CONFIG.SIMULATOR.AGENT_0.SENSORS = ["DEPTH_SENSOR"]
    config.freeze()
elif model_type == "predicted_depth":
    config.defrost()
    config.TASK_CONFIG.SIMULATOR.AGENT_0.SENSORS = ["RGB_SENSOR"]
    config.freeze()
else:
    assert False
    
if sim_eval_mode == "challenge_sim":
    config.defrost()
    config.TASK_CONFIG.SIMULATOR.HABITAT_SIM_V0.ALLOW_SLIDING = True
    config.freeze()
elif sim_eval_mode == "test_sim":
    config.defrost()
    config.TASK_CONFIG.SIMULATOR.HABITAT_SIM_V0.ALLOW_SLIDING = False
    config.freeze()
else:
    assert False
    
config.defrost()
config.TASK_CONFIG.DATASET.CONTENT_SCENES = ["*"]
config.TASK_CONFIG.DATASET.DATA_PATH = "data/datasets/pointnav/gibson/v2/val/val.json.gz" # don't have obstacle_1 scenes, so use Gibson instead
config.freeze()

config.defrost()
config.NUM_ENVIRONMENTS = 1
config.freeze()

config.defrost()
if args.sensors == "":
    config.SENSORS = []
else:
    config.SENSORS = args.sensors.split(",")
# TODO(akadian): collisions are not working
# config.TASK_CONFIG.TASK.MEASUREMENTS.append("COLLISIONS")
config.freeze()

print(config)
print()



#
# create device
#

device = (
    torch.device("cuda:{}".format(config.TORCH_GPU_ID))
    if torch.cuda.is_available()
    else torch.device("cpu")
)

print(device)
print()



#
# construct a single env instead of multiple envs for simplicity
#

env = NavRLEnv(config)



#
# get starting episode and scene so we can reset the environment's episode iterator
#

initial_episode_id = env.current_episode.episode_id
initial_scene_id = env.current_episode.scene_id

print(initial_scene_id, initial_episode_id)
print()

observation = env.reset()

print(env.current_episode.scene_id, env.current_episode.episode_id)
print()



def load_model(
    path,
    observation_space,
    action_space,
    hidden_size,
    normalize_visual_inputs,
    backbone,
    num_recurrent_layers,
    device,
):

    model = PointNavResNetPolicy(
        observation_space=observation_space,
        action_space=action_space,
        hidden_size=hidden_size,
        normalize_visual_inputs=normalize_visual_inputs,
        backbone=backbone,
        num_recurrent_layers=num_recurrent_layers,
        goal_sensor_uuid="pointgoal_with_gps_compass",
    )

    model.to(device)

    saved_model = torch.load(path, map_location=device)
    saved_model_state_dict = {}
    for k, v in saved_model["state_dict"].items():
        new_k = k.replace("actor_critic.", "")
        saved_model_state_dict[new_k] = v

    model.load_state_dict(saved_model_state_dict)

    model_params = 0
    for k,v in model.state_dict().items():
        # print(k, torch.numel(v))
        model_params += torch.numel(v)
    print(model_params)

    saved_model_params = 0
    for k,v in saved_model["state_dict"].items():
        # print(k, torch.numel(v))
        saved_model_params += torch.numel(v)
    print(saved_model_params)

    return model



def eval_model(model_path, num_episodes=-1, max_num_actions_per_episode=10000):

    global observation
    
    print("Resetting env to initial_episode_id and initial_scene_id...")
    print()

    while env.current_episode.episode_id != initial_episode_id or env.current_episode.scene_id != initial_scene_id:
        observation = env.reset()
        
    print()
    print(env.current_episode.scene_id, env.current_episode.episode_id)
    print()
    
    #
    # load model
    #

    if model_type == "rgb":
        observation_space = env.observation_space
    elif model_type == "depth":
        observation_space = env.observation_space
    elif model_type == "predicted_depth":
        observation_space = \
            gym.spaces.Dict({"depth": gym.spaces.Box(low=0.0, high=1.0, shape=(256,256,1), dtype=np.float32),
                             "pointgoal_with_gps_compass": gym.spaces.Box(
                                 low=env.observation_space["pointgoal_with_gps_compass"].low,
                                 high=env.observation_space["pointgoal_with_gps_compass"].high,
                                 shape=env.observation_space["pointgoal_with_gps_compass"].shape,
                                 dtype=env.observation_space["pointgoal_with_gps_compass"].dtype)})
    else:
        assert False

    model = load_model(
        path=model_path,
        observation_space=observation_space,
        action_space=env.action_space,
        hidden_size=args.hidden_size,
        normalize_visual_inputs=bool(args.normalize_visual_inputs),
        backbone=args.backbone,
        num_recurrent_layers=args.num_recurrent_layers,
        device=device,
    )

    model.eval()

    if model_type == "predicted_depth":

        # load depth estimation model
        depth_prediction_model_dict = torch.load(depth_prediction_model_path, map_location=device)
        depth_prediction_model = depth_prediction_model_dict["model"]
        depth_prediction_model.eval() # switch to evaluate mode

    #
    # initialization before main loop
    #

    metric_name = config.TASK_CONFIG.TASK.MEASUREMENTS[0]
    metric_cfg = getattr(config.TASK_CONFIG.TASK, metric_name)
    measure_type = baseline_registry.get_measure(metric_cfg.TYPE)
    metric_uuid = measure_type(None, None)._get_uuid()

    assert measure_type is not None, "invalid measurement type {}".format(metric_cfg.TYPE)

    print(metric_name)
    print(metric_cfg)
    print(measure_type)
    print(metric_uuid)
    print()

    print(len(env.episodes))
    print()

    print(env.current_episode)
    print()

    print(env._env.get_metrics())
    print()

    observations = [observation]
    batch = batch_obs(observations, device)

    num_processes = 1

    test_recurrent_hidden_states = torch.zeros(
        model.net.num_recurrent_layers,
        num_processes,
        args.hidden_size,
        device=device,
    )
    prev_actions = torch.zeros(
        num_processes, 1, device=device, dtype=torch.long
    )
    not_done_masks = torch.zeros(num_processes, 1, device=device)
    print(not_done_masks)

    current_episode_num_actions = 0
    current_episode_reward = 0.0
    current_episode_stats_actions = defaultdict(int)

    stats_episodes = dict()  # dict of dicts that stores stats per episode

    #
    # main loop
    #

    max_num_actions_per_episode = 10000
    
    if num_episodes == -1:
        num_episodes = len(env.episodes)

    while len(stats_episodes) < num_episodes:

        #
        # main loop: choose action
        #

        if model_type == "predicted_depth":
            batch_rgb = batch["rgb"].permute(0,3,1,2) / 255.0 # depth prediction model expects N,C,H,W and values in [0,1]
            depth_pred = depth_prediction_model(batch_rgb)
            batch["depth"] = torch.clamp(depth_pred/max_depth, 0.0, 1.0).permute(0,2,3,1) # action model expects N,H,W,C and values in [0,1]
            batch.pop("rgb") # remove "rgb" key

            tmp_depth = torch.clone(batch["depth"]).detach().numpy()

            # print(np.min(tmp_depth))
            # print(np.max(tmp_depth))
            # print(np.all(np.isfinite(tmp_depth)))

        with torch.no_grad():
            _, actions, _, test_recurrent_hidden_states = model.act(
                batch,
                test_recurrent_hidden_states,
                prev_actions,
                not_done_masks,
                deterministic=False,
            )

            prev_actions.copy_(actions)

        assert len(actions) == 1
        action = actions[0]

        # print(env.habitat_env.task.get_action_name(action.item()))

        #
        # main loop: perform action
        #

        observation, reward, done, info = env.step(action=action.item())

        #
        # main loop: update state
        #

        observations = [observation]
        batch = batch_obs(observations, device)

        dones = [done]
        not_done_masks = torch.tensor(
            [[0.0] if done else [1.0] for done in dones],
            dtype=torch.float,
            device=device,
        )

        current_episode_num_actions += 1
        current_episode_reward += reward
        current_episode_stats_actions[action.item()] += 1

        assert current_episode_num_actions < max_num_actions_per_episode

        if done:

            # record stats
            stats_episode = dict(info)
            stats_episode["reward"] = current_episode_reward
            stats_episode["stats_actions"] = dict(current_episode_stats_actions)

            # if len(stats_episodes) % 100 == 0:
            #     print("Episodes finished: {}".format(len(stats_episodes)))

            print("Episodes finished: {}".format(len(stats_episodes)))
            print(stats_episode)
            print()
            
            stats_episodes[ (env.current_episode.scene_id, env.current_episode.episode_id) ] = stats_episode

            # reset env
            observation = env.reset()
            observations = [observation]
            batch = batch_obs(observations, device)

            test_recurrent_hidden_states = torch.zeros(
                model.net.num_recurrent_layers,
                num_processes,
                args.hidden_size,
                device=device,
            )
            prev_actions = torch.zeros(
                num_processes, 1, device=device, dtype=torch.long
            )
            not_done_masks = torch.zeros(num_processes, 1, device=device)

            current_episode_num_actions = 0
            current_episode_reward = 0.0
            current_episode_stats_actions = defaultdict(int)
            
    return stats_episodes



#
# evaluate model
#

data_dir = os.path.join(srcc_dir, "data")
if not os.path.exists(data_dir): os.makedirs(data_dir)

pickle_file = os.path.join(data_dir, sim_name + "_" + sim_eval_mode + "_" + model_name + ".pickle")

print("Model file: " + model_path)
print("")

print("Pickle file: " + pickle_file)
print("")

stats_episodes = eval_model(model_path)

print("Saving to: " + pickle_file)
print("")
with open(pickle_file, "wb") as p:
    pickle.dump(stats_episodes, p)

print("Finished.")
print()



#
# output success and SPL
#

valid = np.array([ np.isfinite(s[1]["distance_to_goal"]) for s in stats_episodes.items() ])

print(np.count_nonzero(valid) / valid.shape[0])
print(np.count_nonzero(valid))
print(valid.shape[0])

success = np.array([ s[1]["success"] for s in stats_episodes.items() ])
spl = np.array([ s[1]["spl"] for s in stats_episodes.items() ])

success = success[valid]
spl = spl[valid]

print(np.mean(success))
print(np.mean(spl))
