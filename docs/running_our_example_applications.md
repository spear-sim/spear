# Running our Example Applications

## Assumptions

We will assume that you have completed all the steps in our [Getting Started](docs/getting_started.md) tutorial.

## Configure the behavior of `spear`

In typical use cases, you will need to configure the behavior of the `spear` Python package before you interact with it. In each of our example applications, we include a configuration file named `user_config.yaml.example` to use as a starting point. To run each example application, you must rename this file to `user_config.yaml` and modify the contents appropriately for your system. At a minimum, you will need to set the `SPEAR.INSTANCE.GAME_EXECUTABLE` parameter to the location of your `SpearSim` executable. Depending on your platform, the path to your executable should be formatted as follows.

```
Windows: path/to/Windows/SpearSim/Binaries/Win64/SpearSim-Cmd.exe
macOS:   path/to/Mac/SpearSim.app
Linux:   path/to/Linux/SpearSim.sh
```

Your `user_config.yaml` file only needs to specify the value of a parameter if it differs from the defaults defined in the `python/spear/config` directory. You can browse this directory for a complete set of all user-configurable parameters.

If you're running on Linux, you may need to set the `SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES` parameter to an appropriate value for your specific hardware setup. This parameter only has an effect on Linux, and is used to force the Vulkan runtime to load a vendor-specific GPU driver by setting the `VK_ICD_FILENAMES` environment variable. This parameter may or may not be necessary, depending on your specific hardware setup. If you have already set the `VK_ICD_FILENAMES` environment variable before interacting with the `spear` Python package, you do not need to specify `SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES`. If you have an NVIDIA GPU, you probably need to set `SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES` to `/usr/share/vulkan/icd.d/nvidia_icd.json`.

## Run an example application

You are now ready to run an example application.

```console
python examples/getting_started/run.py
```

We recommend browsing through our example applications to get a sense of what is currently possible with SPEAR.
  - [`examples/control_car`](../examples/control_car) demonstrates how to control the default Unreal car.
  - [`examples/control_character`](../examples/control_character) demonstrates how to control the default Unreal humanoid character.
  - [`examples/control_editor`](../examples/control_editor) demonstrates how to control the Unreal Editor and a play-in-editor simulation.
  - [`examples/control_simple_agent`](../examples/control_simple_agent) demonstrates how to control a simple agent and obtain egocentric visual observations.
  - [`examples/enhanced_input`](../examples/enhanced_input) demonstrates how to interact with Unreal's Enhanced Input system.
  - [`examples/getting_started`](../examples/getting_started) demonstrates how to spawn an object and access object properties.
  - [`examples/getting_started_editor`](../examples/getting_started_editor) demonstrates how to spawn an object using the Unreal Editor's built-in Python API.
  - [`examples/getting_started_notebook`](../examples/getting_started_notebook) demonstrates how to interoperate with Jupyter notebooks.
  - [`examples/import_stanford_dataset`](../examples/import_stanford_dataset) demonstrates how to import custom objects.
  - [`examples/movie_render_queue`](../examples/movie_render_queue) demonstrates how to interact with Unreal's Movie Render Queue system.
  - [`examples/mujoco_interop`](../examples/mujoco_interop) demonstrates how to interoperate with the MuJoCo physics engine.
  - [`examples/numpy_interop`](../examples/numpy_interop) demonstrates how to efficiently pass NumPy  MuJoCo physics engine.
  - [`examples/open_level`](../examples/open_level) demonstrates how to dynamically change levels.
  - [`examples/render_dataset`](../examples/render_dataset) demonstrates how to generate random camera poses and render a collection of images.
  - [`examples/render_grid`](../examples/render_grid) demonstrates how to render from a multi-camera rig.
  - [`examples/render_image`](../examples/render_image) demonstrates how to spawn a camera sensor object and render an image.
  - [`examples/render_image_editor`](../examples/render_image) demonstrates how to render an image using the Unreal Editor's built-in Python API.
  - [`examples/sample_nav_mesh`](../examples/sample_nav_mesh) demonstrates how to sample points and shortest paths from Unreal's nav mesh system.
