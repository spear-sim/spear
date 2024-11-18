# Imitation Learning with OpenBot

In this example application, we demonstrate how to collect training data that can be plugged into the [OpenBot](http://www.openbot.org) framework and used to train a navigation policy.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial.

### Important configuration options

You can control the behavior of this example by setting the following parameters in your `user_config.yaml` file, e.g.,
  - `SPEAR.PAKS_DIR` is the directory containing scene data in the form of PAK files.
  - `SP_SERVICES.LEGACY.VEHICLE_AGENT.CAMERA.RENDER_PASSES` can be set to a list of image modalities that you want the agent to return (e.g., setting the value `["depth", "final_color", "segmentation"]` will return depth images, photorealistic RGB images, and segmentation images).

### Running the example

You can run this example as follows.

```console
# generate navigation episodes
python generate_episodes.py --episodes_dir episodes --split train
python generate_episodes.py --episodes_dir episodes --split test

# execute episodes using a global path planner and save data to use during training
python generate_dataset.py --episodes_file episodes/train_episodes.csv --split train

# optional: train a navigation policy using the OpenBot framework (see the OpenBot GitHub repository)

# execute navigation episodes using a trained policy
python run_trained_policy.py --episodes_file episodes/test_episodes.csv
```

Running `generate_episodes.py` will generate navigation episodes and store them in a CSV file. This tool accepts several optional command-line arguments that can be used to control its behavior (see the source code for details), e.g.,
  - `--num_episodes_per_scene` can be used to set the number of episodes generated per scene. Our default in this example is 10, but you should increase this to 1000 if you intend to train your own navigation policy.
  - `--num_candidates_per_episode` can be used to set the number of candidate points you want to sample per episode. Our default in this example is 10, but you can increase it depending on your use case. In the end, one trajectory per episode is selected.
  - `--episodes_dir` and `--split` can be used to output episodes to a specific CSV file at `episodes_dir/split_episodes.csv`.

Running `generate_dataset.py` will generate a dataset of goals, observations, and actions for each episode. The structure of the generated dataset precisely mimics the structure expected by the OpenBot framework, and can therefore be plugged into the OpenBot training code directly. This tool accepts several optional command-line arguments that can be used to control its behavior (see the source code for details), e.g.,
  - `--episodes_file` can be used to read episodes from a specific CSV file.
  - `--create_videos` can be used to generate videos from OpenBot observations. If you use this optional argument, the `ffmpeg` command-line tool must be visible on your path.
  - `--benchmark` can be used to test the overall speed of the simulation.

Running `run_trained_policy.py` will execute a trained navigation policy on a set of navigation episodes. This tool accepts most of the same command-line arguments listed above for `generate_dataset.py`, as well as an optional `--policy_file` argument that can be used to load a specific navigation policy. We include a pre-trained policy with this example application.

### Optional: training a navigation policy

To train a navigation policy with the data you collected, follow the steps provided [here](https://github.com/isl-org/OpenBot/tree/master/policy#policy-training). The datasets generated in this example precisely mimic the structure expected by the OpenBot framework, and can therefore be plugged into the OpenBot training code directly. This step is optional because we include a pre-trained model.
