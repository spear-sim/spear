# Control a Simple Agent

In this example application, we demonstrate how to control a simple sphere agent or an OpenBot agent and obtain egocentric visual observations.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial.

### Important configuration options

You can control the behavior of this example by setting the following parameters in your `user_config.yaml` file, e.g.,
  - `SP_SERVICES.LEGACY_SERVICE.AGENT` can be set to `"SphereAgent"` or `"VehicleAgent"` to change the type of agent in the simulation.
  - `SP_SERVICES.LEGACY.SPHERE_AGENT.CAMERA.RENDER_PASSES` and `SP_SERVICES.LEGACY.VEHICLE_AGENT.CAMERA.RENDER_PASSES` can be set to a list of image modalities that you want the agent to return (e.g., setting the value `["depth", "final_color"]` will return depth images and photorealistic RGB images).

### Running the example

You can run this example as follows.

```console
python run.py
```

 This tool accepts an optional `--benchmark` command-line argument that can be used to measure the overall speed of the simulation.

You should see a game window appear, as well as an OpenCV window that shows egocentric visual observations from the agent. To advance the simulation, press any key while the OpenCV window is in focus.
