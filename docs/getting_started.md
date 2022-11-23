# Getting Started

This tutorial is intended for SPEAR developers that are setting up their development environment for the first time.

## Assumptions 

We will assume for simplicity that you are developing on macOS, although most of these steps map straightforwardly across platforms. We will also assume that you're using Anaconda Python to manage your Python environment, and you have CMake installed. Finally, we will also assume that you have cloned this entire repository including all submodules. All `cd` commands in this tutorial are specified relative to the top-level repository directory.

## Install the Unreal Engine

We recommend installing the Unreal Engine version 4.26 via the Epic Games Launcher, rather than building it from source. This approach is recommended because you'll need the Epic Games Launcher anyway to access art assets from the Unreal Engine Marketplace. You may need to disconnect from your VPN or proxy server when running the Epic Games Launcher. When you install the Unreal Engine, make sure you select _Editor symbols for debugging_ from the list of optional components.

If you're working on Linux, you will need to build the Unreal Engine from source. See [this tutorial](https://docs.unrealengine.com/4.26/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/) for details.

## Install XCode

In order to open and run our example Unreal projects (and for general Unreal plugin development), you need to install a specific version of XCode that matches your Unreal Engine version. For Unreal Engine version 4.26, we have verified that XCode 13.0 behaves as expected. See [this tutorial](https://github.com/botman99/ue4-xcode-vscode-mac) for details.

## Install the spear Python module

```console
# create environment
conda create --name spear-env python=3.8
conda activate spear-env

# install pip
conda install -c anaconda pip

# install the spear Python package
pip install -e python

# install msgpack-rpc-python (do this separately from other Python dependencies so we can use a specific commit from the msgpack-rpc-python GitHub repo)
pip install -e third_party/msgpack-rpc-python
```

## Build third-party C++ libraries

Our Unreal projects require you to build several third-party C++ libraries. We provide a command-line tool for this purpose. The number of parallel jobs in this command should be adjusted based on your own machine specifications.

```console
cd tools
python build_third_party_libs.py --num_parallel_jobs 8
```

## Create symbolic links

Our Unreal projects require several symbolic links to function correctly. We provide a command-line tool to create these links.

```console
cd tools
python create_symbolic_links.py
```

## Generate a config file for your first Unreal project

Our Unreal projects assume that all of their required parameters are declared in a config file. We usually launch our projects via high-level Python code, and this Python code takes care of generating the appropriate config file automatically. However, a valid config file is also required when building our projects, and we must generate this config file explicitly before attempting to build. A valid config file is also required if you want to launch one of our projects directly from the Unreal Editor. To generate a config file, run the following command-line tool.

```console
cd tools
python generate_config.py --unreal_project_dir path/to/spear/cpp/unreal_projects/SpearSim
```

## Launch your first Unreal project

At this point, you should be able to double-click on `cpp/unreal_projects/SpearSim/SpearSim.uproject`, which will open the project in the Unreal Editor, and you should be able to run it successfully.

## Build your first standalone executable

Even though it is possible to launch our projects directly from the Unreal Editor, it is often preferable to build a standalone executable.

```console
# build, cook, stage, package, archive
path/to/UnrealEngine/UE_4.26/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project=path/to/spear/cpp/unreal_projects/SpearSim/SpearSim.uproject -build -cook -stage -package -archive -pak -targetPlatform=Mac -target=SpearSim -clientconfig=Development -archivedirectory=path/to/spear/cpp/unreal_projects/SpearSim/Standalone-Development
```

This step will build a standalone executable at the path `cpp/unreal_projects/SpearSim/Standalone-Development/MacNoEditor/SpearSim.app`. 

### Helpful command-line options

- You can replace `-build` with `-skipbuild`, `-cook` with `-skipcook`, and `-stage -package -archive` with `-skipstage -skippackage -skiparchive`. After doing a complete `-build -cook -stage -package -archive`, you only need to `-cook` if you have edited the `.uproject` in the Unreal Editor, and you only need to `-stage -package -archive` if you want to update the standalone executable in `-archivedirectory`
- If you specify `-skipcook`, you can also specify `-nocompileeditor`, which saves time by not building a special executable that is only required when cooking.
- If you specify `-skipstage -skippackage -skiparchive`, you don't need to specify `-archivedirectory`.
- You can replace `Development` with `Shipping` to build a more optimized executable.
- You can specify `-clean` to do a clean build.
- You can specify `-verbose`, `-UbtArgs="-verbose"`, and `-UbtArgs="-VeryVerbose"` to see additional build details (e.g., the exact command-line arguments that Unreal uses when invoking the underlying compiler).

## Run your first standalone executable

At this point, you should be able to run your standalone executable directly as follows.

```console
# generate config directly inside SpearSim.app
cd tools
python generate_config.py --user_config_files path/to/spear/cpp/unreal_projects/SpearSim/user_config.yaml --output_unreal_project_dir path/to/spear/cpp/unreal_projects/SpearSim/Standalone-Development/MacNoEditor/SpearSim.app/Contents/UE4/SpearSim

# run the executable from the terminal (or double-click on SpearSim.app)
path/to/spear/cpp/unreal_projects/SpearSim/Standalone-Development/MacNoEditor/SpearSim.app/Contents/MacOS/SpearSim
```

## Control the environment via Python

At this point, you should be able to control the environment in an interactive IPython session. Here is a minimal example program that you should be able to execute one line at a time from IPython. We also provide a `run.py` script with each of our examples that executes similar code.

```python
import numpy as np
import os
import spear

# load config (this function will load the parameter values specified in user_config.yaml, as well as sensible defaults for all other parameters)
config = spear.get_config(user_config_files=[ os.path.join(spear.SPEAR_ROOT_DIR, "..", "..", "unreal_projects", "SpearSim", "user_config.yaml") ])

# create Env object
env = spear.Env(config=config)

# reset the simulation to get an initial observation
obs = env.reset()
print(obs["camera_final_color"].shape, obs["camera_final_color"].dtype)

# take a few steps (you should see the sphere move in the Unreal game window)
for i in range(10):
    obs, reward, done, info = env.step(action={"apply_force": np.array([1, 1], dtype=np.float32)})
    print(obs["camera_final_color"].shape, obs["camera_final_color"].dtype, reward, done, info)

# close the environment
env.close()
```
