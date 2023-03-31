# Getting Started

## Minimum system specifications

The minimum hardware and software specifications for the Unreal Engine are given [here](http://docs.unrealengine.com/4.26/en-US/Basics/RecommendedSpecifications).

## Precompiled binaries

The easiest way to start working with SPEAR is to download a precompiled binary for your platform. Our precompiled binaries come pre-packaged with one of our indoor scenes. You can start interactively navigating around this scene with the keyboard and mouse simply by running the downloaded binary with no additional arguments. See our latest release notes for download links.

## Working with multiple scenes

If you'd like to work with multiple scenes, you'll need to download them separately. See our latest release notes for download links. You'll also need to use one of our command-line tools, which means you'll need to follow the steps below.

### Clone this repository including submodules

Our next step is to clone this repository including submodules. We have found that the _recurse submodules_ features in some Git applications don't always download submodules as expected. We therefore recommend using the following commands.

```console
git clone --recurse-submodules https://github.com/isl-org/spear path/to/spear

# checkout the code corresponding to a specific release
cd path/to/spear
git checkout v0.2.0
```

### Install the `spear` Python package

Our next step is to install the `spear` Python package as follows.

```console
# create environment
conda create --name spear-env python=3.8
conda activate spear-env

# install pip
conda install -c anaconda pip

# install msgpack-rpc-python separately from other Python dependencies so
# we can use a specific commit from the msgpack-rpc-python GitHub repository
pip install -e third_party/msgpack-rpc-python

# install the spear Python package
pip install -e python
```

### Navigate around a specific scene

At this point, you can use our `run_executable.py` command-line tool to select which scene you want to navigate.

```console
# interactively navigate through a specific scene
python tools/run_executable.py --executable path/to/executable --paks_dir path/to/paks --scene_id <scene_id>
```

This command-line tool requires the following arguments.
  - `executable` is the path to the executable you downloaded, i.e., `path/to/SpearSim-Mac-Shipping-Cmd.exe` on Windows, `path/to/SpearSim-Mac-Shipping.app` on macOS, or `path/to/SpearSim.sh` on Linux.
  - `paks_dir` is the directory containing the scene data you downloaded.
  - `scene_id` is the name of the scene you want to navigate around. It must be set to one of `starter_content_0000`, `kujiale_0000`, or `warehouse_0000`. If you specify `kujiale_0000`, you must also specify the argument `--map_id kujiale_0000_bake`.

## Interacting with SPEAR programmatically

We provide several example applications that demonstrate how to interact with SPEAR programmatically, and highlight what is currently possible with SPEAR. In order to run our example applications, you will need to follow the steps below.

### Configuring the behavior of SPEAR

In typical use cases, you will need to configure the behavior of SPEAR before you interact with it. In each of our example applications, we include a configuration file named `user_config.yaml.example` to use as a starting point. To run each example application, you must rename this file to `user_config.yaml` and modify the contents appropriately for your system. In all cases, you will need to set the `SPEAR.STANDALONE_EXECUTABLE` parameter to the location of your `SpearSim` binary. Your `user_config.yaml` file only needs to specify the value of a parameter if it differs from the defaults defined in the `python/config` directory. You can browse this directory for a complete set of all user-configurable parameters.

### Run an example applications

We are now ready to run an example application.

```console
python examples/getting_started/run.py
```

We recommend browsing through each of our example applications to get a sense of what is currently possible with SPEAR.
  - [`examples/getting_started`](examples/getting_started) demonstrates how to control a simple sphere agent or an OpenBot agent and obtain egocentric visual observations.
  - [`examples/generate_image_dataset`](examples/generate_image_dataset) demonstrates how to generate a static image dataset using our freeform camera agent.
  - [`examples/imitation_learning_openbot`](examples/imitation_learning_openbot) demonstrates how to collect training data that can be plugged into the OpenBot framework and used to train a navigation policy.
  - [`examples/open_loop_control_fetch`](examples/open_loop_control_fetch) demonstrates how to control a Fetch agent to pick up an object and move it to another location.
