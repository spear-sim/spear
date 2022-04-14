# Getting Started

This tutorial is intended for InteriorSim developers that are setting up their environment for the first time.

## Assumptions 

We will assume for simplicity that you are developing on macOS, although most of these steps map straightforwardly across platforms. We will also assume that you're using Anaconda Python to manage your Python environment, and you have CMake installed. Finally, we will also assume that you have cloned this entire repository including all submodules. All `cd` commands in this tutorial are specified relative to the top-level repository directory.

## Install the Unreal Engine

We recommend installing the Unreal Engine version 4.26 via the Epic Games Launcher, rather than building it from source. This approach is recommended because you'll need the Epic Games Launcher anyway to access art assets from the Unreal Engine Marketplace. You may need to disconnect from your VPN or proxy server when running the Epic Games Launcher. When you install the Unreal Engine, make sure you select _Editor symbols for debugging_ from the list of optional components.

If you're working on Linux, you will need to build the Unreal Engine from source. See [this tutorial](https://docs.unrealengine.com/4.26/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/) for details.

## Install XCode

In order to open and run our example Unreal projects (and for general Unreal plugin development), you need to install a specific version of XCode that matches your Unreal Engine version. For Unreal Engine version 4.26, you must install XCode 11.1. See [this tutorial](https://github.com/botman99/ue4-xcode-vscode-mac) for details.

## Build third-party C++ libraries

Our Unreal projects require you to build several third-party C++ libraries. We provide a command-line tool for this purpose.

```console
cd code/tools
python build_third_party_libs.py
```

## Create symbolic links

Our Unreal projects depend on several of our Unreal plugins. Let's create symbolic links to these plugins.

```console
cd code/tools
python create_symbolic_links.py
```

## Install the interiorsim Python package

```console
# create environment
conda create --name interiorsim-env
conda activate interiorsim-env

# install pip
conda install -c anaconda pip

# install msgpack-rpc-python separately from other Python dependencies
pip install -e code/third_party/msgpack-rpc-python

# install interiorsim
pip install -e code/python_module
```

## Launch your first Unreal project

Our Unreal projects assume that all of their required parameters are declared in a config file. We usually launch our projects via high-level Python code, and this Python code takes care of generating the appropriate config file automatically. However, if you want to launch one of our projects directly from the Unreal Editor, you will need to generate the config file in a separate step. We provide a command-line tool for this purpose.

```console
cd code/tools
python generate_config.py --config_files path/to/interiorsim/code/unreal_projects/PlayEnvironment/user_config.yaml --output_unreal_project_dir path/to/interiorsim/code/unreal_projects/PlayEnvironment
```

At this point, you should be able to double-click on `code/unreal_projects/PlayEnvironment/PlayEnvironment.uproject`, which will open the project in the Unreal Editor, and you should be able to run it successfully.

Our other examples require you to download additional content before you can run them. See the README file in each example directory for more details.

## Build your first standalone executable

This step is not strictly necessary, because you can run our example environments directly from the Unreal Editor. That being said, the Unreal Editor is a heavyweight application, so it is often preferable to build a standalone executable.

```console
# build, cook, stage, package, archive
path/to/UnrealEngine/UE_4.26/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project=path/to/interiorsim/code/unreal_projects/PlayEnvironment/PlayEnvironment.uproject -build -cook -stage -package -archive -targetPlatform=Mac -target=PlayEnvironment -clientconfig=Development -archivedirectory=path/to/interiorsim/code/unreal_projects/PlayEnvironment/Standalone-Development
```

This step will build a standalone executable at the path `code/unreal_projects/PlayEnvironment/Standalone-Development/MacNoEditor/PlayEnvironment.app`.

You can replace `-build` with `-skipbuild`, `-cook` with `-skipcook`, and `-stage -package -archive` with `-skipstage -skippackage -skiparchive`. After doing a complete `-build -cook -stage -package -archive`, you only need to `-cook` if you have edited the `.uproject` in the Unreal Editor, and you only need to `-stage -package -archive` if you want to update the standalone executable in `-archivedirectory`. If you specify `-skipstage -skippackage -skiparchive`, you don't need to specify `-archivedirectory`. You can replace `Development` with `Shipping` to build a more optimized executable.

## Run your first standalone executable

At this point, you should be able to run your standalone executable directly as follows.

```
# generate config
cd utils
python generate_config.py --config_files path/to/interiorsim/code/unreal_projects/PlayEnvironment/user_config.yaml --output_unreal_project_dir path/to/interiorsim/code/unreal_projects/PlayEnvironment/Standalone-Development/MacNoEditor/PlayEnvironment.app/Contents/UE4/PlayEnvironment

# run the executable in the terminal, alternatively you can double-click on PlayEnvironment.app
path/to/interiorsim/code/unreal_projects/PlayEnvironment/Standalone-Development/MacNoEditor/PlayEnvironment.app/Contents/MacOS/PlayEnvironment
```

## Control the environment via Python

At this point, you should be able to control the environment in an interactive IPython session. Here is a minimal example program that you should be able to execute one line at a time from IPython. We also provide a `run.py` script with each of our examples that executes similar code.

```python
import numpy as np
import os

import interiorsim
import interiorsim.config
from interiorsim.constants import INTERIORSIM_ROOT_DIR

config_files = []
config_files.append(os.path.join(INTERIORSIM_ROOT_DIR, "../../unreal_projects/PlayEnvironment/user_config.yaml"))

# you can append your own user-specific config file(s) to config_files to override
# default values; your config file(s) only need to specify a value if you want to
# change it from its default
# config_files.append(os.path.join("path", "to", "my", "user_config.yaml")

config = interiorsim.config.get_config(config_files)

# allow editing of config values
config.defrost()

# standalone executable mode
config.INTERIORSIM.LAUNCH_MODE = "standalone_executable"
config.INTERIORSIM.STANDALONE_EXECUTABLE = "path/to/interiorsim/code/unreal_projects/PlayEnvironment/Standalone-Development/MacNoEditor/PlayEnvironment.app"

# uproject mode
# config.INTERIORSIM.LAUNCH_MODE = "uproject"
# config.INTERIORSIM.UPROJECT = "path/to/interiorsim/code/unreal_projects/PlayEnvironment/PlayEnvironment.uproject"

# prevent further editing of config values
config.freeze()

# triggers several "WARNING:tornado.general:Connect error on fd 26: ECONNREFUSED" warnings which can be ignored
env = interiorsim.Env(config)

# get action space and observation for the agent
print(f"action space - {env.action_space}")
print(f"observation space - {env.observation_space}")

# resets the simulation, required before getting observations
obs = env.reset()

# take a few actions; in this example, each action is specified as a 2D point, you should see the ball move in the Unreal game window
for i in range(10):
    obs, reward, done, _ = env.step({"apply_force": [1, 1]})

# close the environment
env.close()
```
