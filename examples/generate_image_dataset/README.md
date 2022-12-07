# Generate Image Dataset

In this example application, we demonstrate how to generate an image dataset using our freeform camera agent.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our top-level [README](http://github.com/isl-org/spear).

### Important configuration options

You can control the behavior of this example by setting the following parameters in your `user_config.yaml` file, e.g.,
  - `SPEAR.DATA_DIR` is the directory containing our test scene.
  - `SPEAR.CONTENT_DIR` is the `Content` directory corresponding to your precompiled `SpearSim` binary.
  - `SIMULATION_CONTROLLER.SPHERE_AGENT.CAMERA_PASSES` and `SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA_PASSES` can be set to a list of image modalities that you want the agent to return (e.g., setting the value `["final_color", "depth", "segmentation"]` will return photorealistic RGB images, depth images, and segmentation images).

Your `user_config.yaml` file only needs to specify the value of a parameter if it differs from the defaults defined in the `python/config` directory. You can browse this directory for a complete set of all user-configurable parameters.

### Running the example

You can run the example as follows.

```console
# generate camera poses
python generate_poses.py

# generate images
python generate_images.py
```

Running `generate_poses.py` will generate a `poses.csv` file consisting of camera poses that will be used in the following step. This tool accepts several optional command-line arguments that can be used to control its behavior (see the source code for details).

Running `generate_images.py` will generate images in an `images` directory by default. This tool accepts several optional command-line arguments that can be used to control its behavior (see the source code for details), e.g.,
  - `--poses_file` can be used to generate images based on the camera poses in a specific CSV file.
  - `--rendering_mode` can be set to `"baked"` to use baked lighting, or `"raytracing"` for ray-traced lighting if you are running on Windows and you have a GPU that supports DirectX 12.
  - `--num_internal_steps` can be used to control the image quality when running in ray-traced mode.
  - `--benchmark` can be used to test the overall speed of the simulation.
  - `--wait_for_key_press` can be used to compare the game window output to the image that has been saved to disk.
