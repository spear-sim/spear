# Getting Started

## Assumptions

We will assume that you are developing on a version of Windows, macOS, or Linux that is compatible with Unreal Engine 5.5. We will also assume that you have Git and Anaconda Python installed.

## Clone this repository including submodules

The first step is to clone this repository including submodules. We have found that the _recurse submodules_ features in some Git applications don't always download submodules as expected. We therefore recommend using the following commands.

```console
# clone repository
git clone https://github.com/spear-sim/spear path/to/spear --recurse-submodule

# checkout the code corresponding to a specific release (optional)
cd path/to/spear
git checkout v0.5.0
```

## Install the `spear` Python package

The next step is to install the `spear` Python package as follows.

```console
# create environment
conda create --name spear-env python=3.11
conda activate spear-env

# install mayavi separately from other Python dependencies (Windows only)
conda install -c conda-forge mayavi

# install msgpack-rpc-python separately from other Python dependencies, because we need
# to use a specific commit from a specific fork of the msgpack-rpc-python GitHub repository
pip install -e third_party/msgpack-rpc-python

# install gcc (Linux only)
sudo apt-get install gcc

# install the spear Python package
pip install -e python
```

If you're developing on Linux, you will need to install `gcc` if it isn't already installed on your system. `gcc` is required when installing one of our Python dependencies via `pip`.

## Navigate around a specific scene

At this point, you can use our `run_executable.py` command-line tool to navigate around a specific scene. For example, if you wanted to navigate through our `debug_0000` scene, you would use the following command.

```console
python tools/run_executable.py --map /Game/Spear/Scenes/debug_0000/Maps/debug_0000 --executable path/to/executable
```

When executing our command-line tool, you will need to specify a logical Unreal path to the `--map` you want to navigate around, e.g.,

```
/Game/Spear/Scenes/debug_0000/Maps/debug_0000
```

You can optionally specify a path to an `--executable`. We provide links to precompiled executables in our [release notes](https://github.com/spear-sim/spear/releases/tag/v0.5.0), or you can build an executable from source by following our [Building SpearSim](docs/building_spearsim.md) tutorial. If you don't specify an `--executable`, our command-line tool will assume that you built an executable from source in a default location. Depending on your platform, the path to your `--executable` should be formatted as follows.

```
Windows: path/to/SpearSim-v0.5.0-Win64-Shipping/SpearSim/Binaries/Win64/SpearSim-Win64-Shipping-Cmd.exe
macOS:   path/to/SpearSim-v0.5.0-Mac-Shipping/SpearSim-Mac-Shipping.app
Linux:   path/to/SpearSim-v0.5.0-Linux-Shipping/SpearSim.sh
```

The following command-line arguments are optional.

  - `--pak_files` is a comma-separated list of PAK files to load during startup. It is necessary to specify additional PAK files when specifying a `--map` that isn't included with our `SpearSim` executable.
  - `--vk_icd_filenames` only has an effect on Linux, and is used to force the Vulkan runtime to load a vendor-specific GPU driver. Our `run_executable.py` script will set the `VK_ICD_FILENAMES` environment variable to whatever is passed into `--vk_icd_filenames`. This argument may or may not be necessary, depending on your specific hardware setup. If you have already set the `VK_ICD_FILENAMES` environment variable before invoking `run_executable.py`, you do not need to specify `--vk_icd_filenames`. If you have an NVIDIA GPU, you probably need to specify `--vk_icd_filenames /usr/share/vulkan/icd.d/nvidia_icd.json`.

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
  - [`examples/getting_started`](../examples/getting_started) demonstrates how to spawn an object and access object properties.
  - [`examples/import_stanford_dataset`](../examples/import_stanford_dataset) demonstrates how to import custom objects.
  - [`examples/mujoco_interop`](../examples/mujoco_interop) demonstrates how to interoperate with the MuJoCo physics engine.
  - [`examples/numpy_interop`](../examples/numpy_interop) demonstrates how to efficiently pass NumPy arrays to and from Unreal game entities.
  - [`examples/render_image`](../examples/render_image) demonstrates how to spawn a camera sensor object and render an image.
