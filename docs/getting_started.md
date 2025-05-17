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

## Build a standalone `SpearSim` executable

In order to proceed further, you must build a standalone `SpearSim` executable from source by following our [Building SpearSim](building_spearsim.md) tutorial. After building our `SpearSim` executable, you can start interactively navigating around our default scene with the keyboard and mouse simply by running the executable with no additional arguments.

## Navigate around a specific scene

At this point, you can use our `run_executable.py` command-line tool to navigate around a specific scene. For example, if you wanted to navigate through our `debug_0000` scene, you would use the following command.

```console
python tools/run_executable.py --map /Game/Spear/Scenes/debug_0000/Maps/debug_0000
```

When executing our command-line tool, you will need to specify a logical Unreal path to the `--map` you want to navigate around, e.g.,

```
/Game/Spear/Scenes/debug_0000/Maps/debug_0000
```

You can also optionally specify an `--executable`. If you don't specify an `--executable`, our command-line tool will assume that you built an executable from source in the default location in our [Building SpearSim](building_spearsim.md) tutorial. Depending on your platform, the path to your `--executable` should be formatted as follows.

```
Windows: path/to/Windows/SpearSim/Binaries/Win64/SpearSim-Cmd.exe
macOS:   path/to/SpearSim-Mac-Shipping.app
Linux:   path/to/SpearSim.sh
```

The following command-line arguments are optional.

  - `--pak_files` is a comma-separated list of PAK files to load during startup. It is necessary to specify additional PAK files when specifying a `--map` that isn't included with our `SpearSim` executable.
  - `--vk_icd_filenames` only has an effect on Linux, and is used to force the Vulkan runtime to load a vendor-specific GPU driver. Our `run_executable.py` script will set the `VK_ICD_FILENAMES` environment variable to whatever is passed into `--vk_icd_filenames`. This argument may or may not be necessary, depending on your specific hardware setup. If you have already set the `VK_ICD_FILENAMES` environment variable before invoking `run_executable.py`, you do not need to specify `--vk_icd_filenames`. If you have an NVIDIA GPU, you probably need to specify `--vk_icd_filenames /usr/share/vulkan/icd.d/nvidia_icd.json`.

## Programmatically interact with SPEAR via Python

We provide several example applications that demonstrate how to programmatically interact with SPEAR via Python. In order to run our example applications, you will need to follow the steps in our [Running our Example Applications](running_our_example_applications.md) tutorial.
