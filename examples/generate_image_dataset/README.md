# Generate Image Dataset

In this example application, we demonstrate how to generate an image dataset using our freeform camera agent.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial.

### Important configuration options

You can control the behavior of this example by setting the following parameters in your `user_config.yaml` file, e.g.,
  - `SPEAR.PAKS_DIR` is the directory containing scene data in the form of PAK files.
  - `SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES` can be set to a list of image modalities that you want the agent to return (e.g., setting the value `["depth", "final_color", "segmentation"]` will return depth images, photorealistic RGB images, and segmentation images).

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

Running `generate_images.py` will generate images in an `images` directory. This tool accepts several optional command-line arguments that can be used to control its behavior (see the source code for details), e.g.,
  - `--poses_file` can be used to generate images based on the camera poses in a specific CSV file.
  - `--num_internal_steps` can be used to control the overall image quality, since Unreal aggregates rendering information across multiple frames to compute various rendering effects.
  - `--benchmark` can be used to test the overall speed of the simulation.
  - `--wait_for_key_press` can be used to compare the game window output to the image that has been saved to disk.
