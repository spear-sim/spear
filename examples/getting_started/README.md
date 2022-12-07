# Getting Started

In this example application, we demonstrate how to control a simple sphere agent or an OpenBot agent and obtain egocentric visual observations.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our top-level [README](http://github.com/isl-org/spear).

### Important configuration options

You can control the behavior of this example by setting the following parameters in your `user_config.yaml` file.
  - `SIMULATION_CONTROLLER.AGENT` can be set to `"SphereAgent"` or `"OpenBotAgent"` to change the type of agent in the simulation.
  - `SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA_PASSES` and `SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA_PASSES` can be set to a list of image modalities that you want the agent to return (e.g., setting the value `["final_color", "depth"]` will return photorealistic RGB images and depth images).

Your user_config.yaml file only needs to specify the value of a parameter if it differs from the defaults defined in the `python/config` directory. You can browse this directory for a complete set of all user-configurable parameters.

### Running the example

You can run the example as follows.

```console
python run.py
```

You should see a game window appear, as well as an OpenCV window that shows egocentric observations from the agent. To advance the simulation, press any key while the OpenCV window is in focus. You can run this example with an optional `--benchmark` flag to measure the overall speed of the simulation on your system.
