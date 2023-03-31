# Open-Loop Control with Fetch

In this example application, we demonstrate how control a Fetch agent to pick up an object and move it to another location.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial.

### Important configuration options

You can control the behavior of this example by setting the following parameters in your user_config.yaml file, e.g.,
  - `SPEAR.PAKS_DIR` is the directory containing scene data in the form of PAK files.
  - `SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES` can be set to a list of image modalities that you want the agent to return (e.g., setting the value `["depth", "final_color", "segmentation"]` will return depth images, photorealistic RGB images, and segmentation images).

Your `user_config.yaml` file only needs to specify the value of a parameter if it differs from the defaults defined in the `python/config` directory. You can browse this directory for a complete set of all user-configurable parameters.

### Running the example

You can run the example as follows.

```console
# generate actions
python generate_actions.py

# execute actions
python run.py
```

Running `generate_actions.py` will generate an `actions.csv` file consisting of actions that will be used in the following step. This tool accepts several optional command-line arguments that can be used to control its behavior (see the source code for details), e.g.,

  - `--scene_id` can be used to specify which scene to generate actions for.

Running `run.py` will execute the previously generated actions in an open-loop fashion on the Fetch agent. This tool accepts several optional command-line arguments that can be used to control its behavior (see the source code for details), e.g.,

  - `--action_file` can be used to specify which actions to load.
  - `--scene_id` can be used to specify which scene to load.
  - `--benchmark` can be used to test the overall speed of the simulation.
