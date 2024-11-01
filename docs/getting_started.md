# Getting Started

If you'd like to work with multiple scenes, or programmatically interact with SPEAR via Python, you will need to follow the steps below.

## Clone this repository including submodules

The first step is to clone this repository including submodules. We have found that the _recurse submodules_ features in some Git applications don't always download submodules as expected. We therefore recommend using the following commands.

```console
git clone --recurse-submodules https://github.com/spear-sim/spear path/to/spear

# checkout the code corresponding to a specific release (optional)
cd path/to/spear
git checkout v0.5.0
```

## Install the `spear` Python package

The next step is to install the `spear` Python package as follows.

```console
# create environment
conda create --name spear-env python=3.9
conda activate spear-env

# install msgpack-rpc-python separately from other Python dependencies, because we need
# to use a specific commit from a specific fork of the msgpack-rpc-python GitHub repository
pip install -e third_party/msgpack-rpc-python

# install the spear Python package
pip install -e python
```
 
## Navigate around a specific scene

At this point, you can use our `run_executable.py` command-line tool to select which scene you want to navigate around. If you wanted to navigate through our `debug_0000` scene, you would use the following command.

```console
python tools/run_executable.py --executable path/to/executable --scene_id debug_0000
```

Depending on your platform, you will need to specify the following path to your `--executable`. We provide links to precompiled binaries in our [release notes](https://github.com/spear-sim/spear/releases/tag/v0.5.0).

```
Windows: path/to/SpearSim-v0.5.0-Win64-Shipping/SpearSim/Binaries/Win64/SpearSim-Win64-Shipping-Cmd.exe
macOS:   path/to/SpearSim-v0.5.0-Mac-Shipping/SpearSim-Mac-Shipping.app
Linux:   path/to/SpearSim-v0.5.0-Linux-Shipping/SpearSim.sh
```

You will also need to specify the following command-line arguments.

  - `--scene_id` is the name of the scene you want to navigate around (e.g., `apartment_0000`, `debug_0000`, `kujiale_0000`, `warehouse_0000`, `...`). If you specify a `kujiale` or `warehouse` scene, then you will need to download that scene separately (see below).

The following command-line arguments are optional.

  - `--paks_dir` is the location of your downloaded scene data (see below), and is necessary if you specify a `kujiale` or `warehouse` scene for `--scene_id`.
  - `--vk_icd_filenames` only has an effect on Linux, and is used to force the Vulkan runtime to load a vendor-specific GPU driver. Our `run_executable.py` script will set the `VK_ICD_FILENAMES` environment variable to whatever is passed into `--vk_icd_filenames`. This argument may or may not be necessary, depending on your specific hardware setup. If you have already set the `VK_ICD_FILENAMES` environment variable before invoking `run_executable.py`, you do not need to specify `--vk_icd_filenames`. If you have an NVIDIA GPU, you probably need to specify `--vk_icd_filenames /usr/share/vulkan/icd.d/nvidia_icd.json`.

## Download scene data (optional)

In order to work with a `kujiale` or `warehouse` scene, you will need to download each scene as follows.

```console
python tools/download_paks.py --paks_dir path/to/spear-paks --scene_ids kujiale_0000
```

The `--paks_dir` argument is the top-level directory where scene data will be downloaded. If you want to download all of our scene data, you can omit the `--scene_ids` argument.

## Programmatically interact with SPEAR via Python

We provide several example applications that demonstrate how to programmatically interact with SPEAR via Python, and highlight what is currently possible with SPEAR. In order to run our example applications, you will need to follow the steps below.

In typical use cases, you will need to configure the behavior of SPEAR before you interact with it. In each of our example applications, we include a configuration file named `user_config.yaml.example` to use as a starting point. To run each example application, you must rename this file to `user_config.yaml` and modify the contents appropriately for your system. In typical use cases, you will need to set the `SPEAR.STANDALONE_EXECUTABLE` parameter to the location of your `SpearSim` executable (see the note above for which executable to use, depending on your platform). Your `user_config.yaml` file only needs to specify the value of a parameter if it differs from the defaults defined in the `python/spear/config` directory. You can browse this directory for a complete set of all user-configurable parameters.

If you're running on Linux, you may need to set the `SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES` parameter to an appropriate value for your specific hardware setup. See the note above for a more detailed discussion.

You are now ready to run an example application.

```console
python examples/getting_started/run.py
```

We recommend browsing through each of our example applications to get a sense of what is currently possible with SPEAR.
  - [`examples/control_simple_agent`](../examples/control_simple_agent) demonstrates how to control a simple agent and obtain egocentric visual observations.
  - [`examples/enhanced_input`](../examples/enhanced_input) demonstrates how to interact with Unreal's Enhanced Input system.
  - [`examples/generate_image_dataset`](../examples/generate_image_dataset) demonstrates how to generate a dataset of images using our camera agent.
  - [`examples/getting_started`](../examples/getting_started) demonstrates how to spawn an object and access object properties.
  - [`examples/imitation_learning_openbot`](../examples/imitation_learning_openbot) demonstrates how to collect navigation training data for an OpenBot.
  - [`examples/import_stanford_dataset`](../examples/import_stanford_dataset) demonstrates how to import custom objects.
  - [`examples/mujoco_interop`](../examples/mujoco_interop) demonstrates how to interoperate with the MuJoCo physics engine.
  - [`examples/numpy_interop`](../examples/numpy_interop) demonstrates how to efficiently pass NumPy arrays to and from Unreal game entities.
  - [`examples/open_loop_control_fetch`](../examples/open_loop_control_fetch) demonstrates how to control a Fetch robot agent.
