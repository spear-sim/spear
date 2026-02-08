# Getting Started

## Assumptions

We will assume that you are developing on a version of Windows, macOS, or Linux that is compatible with Unreal Engine 5.5. We will also assume that you're using Anaconda Python to manage your Python environment, and that you have Git and CMake installed.

## Minimum and recommended system specifications

Minimum and recommended system specifications for the Unreal Engine are given [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/hardware-and-software-specifications-for-unreal-engine?application_version=5.5).

## Install the Unreal Engine

We recommend installing the Unreal Engine 5.5 via the Epic Games Launcher, rather than building it from source. We recommend installing to a path that does not contain spaces. You may need to disconnect from your VPN or proxy server when running the Epic Games Launcher. When you install the Unreal Engine, make sure to select _Editor symbols for debugging_ from the list of optional components.

If you're developing on Linux, you will need to download the Unreal Engine from [here](https://www.unrealengine.com/en-US/linux).

Several of our command-line tools require an `--unreal-engine-dir` argument. This argument must point to the top-level directory where you installed the Unreal Engine. Depending on your platform, the default install location will be as follows. However, as noted above, we recommend installing the Unreal Engine to a path that doesn't contain spaces. If you're developing on Linux, you must specify the path to the top-level directory where you unzipped the `Linux_Unreal_Engine_5.5.4.zip` file linked above.

```
Windows: C:\Program Files\Epic Games\UE_5.5
macOS:   /Users/Shared/Epic Games/UE_5.5
Linux:   path/to/Linux_Unreal_Engine_5.5.4
```

## Install an appropriate compiler

If you're developing on Windows or macOS, you will need to install a specific compiler that is compatible with Unreal Engine 5.5. If you're developing on Linux, the Unreal Engine ships with its own version of `clang` and `libc++`, so there is no need to install another compiler to build the `SpearSim` project in this repository, but you will still need to install an appropriate compiler to build our third-party dependencies as described below. We have verified that the following compilers behave as expected when building `SpearSim`.

```
Windows: Visual Studio 2022
macOS:   XCode 16
```

If you're developing on Windows, make sure to select _Desktop development with C++_ and _Game development with C++_ from the _Workloads_ tab when installing Visual Studio, and also make sure to select _MSVC v143 - VS2022 C++ x64/x86 build tools (v14.44-17.14)_ from the _Individual Components_ tab.

## Configure your terminal

If you're developing on Windows, you will need to run our build steps in a terminal that meets the following requirements.

- Your terminal must be able to access Anaconda Python, `cmake`, and the Visual Studio command-line tools.
- You will need to run our build steps from within the Anaconda environment where the `spear` package is installed.

There are multiple possible ways to satisfy these requirements. One possibility is to use the _Developer PowerShell for VS 2022_ profile in the _Terminal_ application to run our build steps. This profile will be installed to the _Terminal_ application when you install Visual Studio. Additionally, you can configure your profile to access your Anaconda environment by opening an _Anaconda PowerShell Prompt_ with administrator privileges (located in your Start Menu after you install Anaconda) and executing the following commands.

```console
# This step will add a block of PowerShell code to C:\Users\username\Documents\WindowsPowerShell\profile.ps1
# to make your Anaconda installation visible to PowerShell.
conda init powershell

# When running as administrator, PowerShell will automatically load C:\Users\username\Documents\WindowsPowerShell\profile.ps1,
# but this step will allow it to load profile.ps1 even when running as a normal user, so Anaconda will always be visible.
Set-ExecutionPolicy -Scope LocalMachine -ExecutionPolicy Bypass
```

After executing these commands, you will be able to use the _Developer PowerShell for VS 2022_ profile in the _Terminal_ application, and it will be able to access Anaconda Python and the Visual Studio command-line tools.

Confusingly, there is also a legacy application on Windows called _Windows PowerShell_ (blue icon) that provides a similar command-line interface to _Terminal_ (black icon). We have found that _Windows PowerShell_ can make the build environment appear as 32-bit instead of 64-bit, which can cause subtle problems for several of our build steps. So our recommended setup is to use an appropriately configured _Developer PowerShell for VS 2022_ profile in the _Terminal_ application (black icon) specifically. With this nuance in mind, we provide a command-line tool to check that your terminal is correctly configured.

```console
# check that terminal is correctly configured (Windows only)
python tools/check_terminal_windows.py
```

## Install Git and CMake

If you have Anaconda installed, but you don't already have Git and CMake, you can easily obtain them by executing the following command.

```console
# create environment
conda create --name spear-env python=3.11
conda activate spear-env

# install git
conda install -c conda-forge git

# install cmake
pip install cmake
```

## Clone this repository including submodules

The next step is to clone this repository including submodules. We have found that the _recurse submodules_ features in some Git applications don't always download submodules as expected. We therefore recommend using the following commands.

```console
# clone repository
git clone https://github.com/spear-sim/spear path/to/spear --recurse-submodules
```

## Install the `spear` Python package

The next step is to install the `spear` Python package as follows.

```console
# create environment
conda create --name spear-env python=3.11
conda activate spear-env

# install mayavi separately from other Python dependencies (Windows only)
conda install -c conda-forge mayavi=4.8.2

# install gcc (Linux only)
sudo apt-get install gcc

# install the spear Python package
pip install -e python
```

If you're developing on Linux, you will need to install `gcc` if it isn't already installed on your system. `gcc` is required when installing one of our Python dependencies via `pip`.

The Unreal Editor has its own Python environment, so you will need to install the `spear` Python package into the Unreal Editor's Python environment in a separate step. We provide a command-line tool for this purpose.

```console
# install the spear Python package into the Unreal Editor Python environment
python tools/install_python_package_in_editor_env.py --unreal-engine-dir path/to/UE_5.5
```

## Build third-party C++ libraries

Our `SpearSim` project and our `spear_ext` Python extension module each require you to build several third-party C++ libraries. We provide a command-line tool for this purpose.

```console
# build third-party libraries (Windows and macOS)
python tools/build_third_party_libs.py

# build third-party libraries (Linux)
sudo apt-get install g++ make
python tools/build_third_party_libs.py --unreal-engine-dir path/to/UE_5.5
```

If you're developing on Linux, you will need to install `g++` and `make` if they aren't already installed on your system. `g++` is required to build the Boost build tool, and `make` is required by CMake to build all third-party libraries other than Boost. Additionally, if you're developing on Linux, you must specify `--unreal-engine-dir`, because we use the version of `clang` and `libc++` that ships with the Unreal Engine to build our third-party libraries.

## Build and install the `spear_ext` Python extension module

Most of the functionality in our `spear` Python package requires you to build and install our `spear_ext` Python extension module. We provide a command-line tool for this purpose.

```console
# build and install the spear_ext Python extension module (Windows and macOS)
python tools/install_python_extension.py

# build and install the spear_ext Python extension module (Linux)
python tools/install_python_extension.py --unreal-engine-dir path/to/UE_5.5
```

If you're developing on Linux, you must specify `--unreal-engine-dir`, because we use the version of `clang` and `libc++` that ships with the Unreal Engine to build our extension module.

## Copy content from the Unreal Engine

Our `SpearSim` project requires you to explicitly copy some content from your Unreal Engine installation to the project directory. We provide a command-line tool for this purpose.

```console
python tools/copy_engine_content.py --unreal-engine-dir path/to/UE_5.5
```

## Build the `SpearSim` project

You are now ready to build the `SpearSim` project as follows.

```console
# minimal build required to open the SpearSim project inside the Unreal Editor (optional)
python tools/run_uat.py --unreal-engine-dir path/to/UE_5.5 -build

# build a standalone executable
python tools/run_uat.py --unreal-engine-dir path/to/UE_5.5 -build -cook -stage -package -archive -pak
```

Our `run_uat.py` tool is a thin wrapper around Unreal's [`RunUAT`](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/Deployment/BuildOperations) tool. Our tool consumes `--unreal-engine-dir`, provides Unreal's `RunUAT` tool with sensible default values for a few commonly used arguments, and otherwise forwards all arguments directly to `RunUAT`. This step will generate a standalone executable at the following locations.

```
Windows: cpp\unreal_projects\SpearSim\Standalone-Development\Windows\SpearSim.exe
macOS:   cpp/unreal_projects/SpearSim/Standalone-Development/Mac/SpearSim.app
Linux:   cpp/unreal_projects/SpearSim/Standalone-Development/Linux/SpearSim.sh
```

When building with our `run_uat.py` tool, you can optionally specify `--build-config Shipping` to build more a optimized standalone executable. In this case, the executable will be generated in `Standalone-Shipping` instead of `Standalone-Development`.

### Helpful `RunUAT` command-line options

- If you want to rebuild after doing a build as described above, you can replace `-build` with `-skipbuild`, `-cook` with `-skipcook`, and `-stage -package -archive` with `-skipstage -skippackage -skiparchive`, depending on what you want to accomplish.
- After specifying `-build` once, you only need to specify `-build` again if you have modified the C++ code or build configuration files.
- After specifying `-cook` once, you only need to specify `-cook` again if you have modified the project in the Unreal Editor.
- After specifying `-stage -package -archive` once, you only need to specify `-stage -package -archive` again if you want to update the standalone executable in `Standalone-Development` or `Standalone-Shipping`.
- You can specify `-skipbuild -skipcook` if you only want to propagate changes in `cpp/unreal_projects/SpearSim/Config` to the standalone executable in `Standalone-Development` or `Standalone-Shipping`.
- You can specify `-nocompileeditor` if you want to skip building the binaries required for cooking and opening the project in the Unreal Editor.
- You can specify `-specifiedarchitecture=arm64+x86_64` to build a universal binary on macOS.
- You can specify `-clean` to do a clean build.

## Navigate around a specific scene

At this point, you can use our `run_executable.py` command-line tool to navigate around a specific scene. For example, if you wanted to navigate through our `debug_0000` scene, you would use the following command.

```console
python tools/run_executable.py --map /Game/Spear/Scenes/debug_0000/Maps/debug_0000
```

When executing our command-line tool, you will need to specify a logical Unreal path to the `--map` you want to navigate around, e.g.,

```
/Game/Spear/Scenes/debug_0000/Maps/debug_0000
```

You can also optionally specify an `--executable`. If you don't specify an `--executable`, our command-line tool will assume that you built an executable from source in the default location (described above). Depending on your platform, if you do provide an executable, the path to your executable should be formatted as follows.

```
Windows: path\to\Standalone-Development\Windows\SpearSim.exe
macOS:   path/to/Standalone-Development/Mac/SpearSim.app
Linux:   path/to/Standalone-Development/Linux/SpearSim.sh
```

The following command-line arguments are optional.

  - `--pak-files` is a comma-separated list of PAK files to load during startup. It is necessary to specify additional PAK files when specifying a `--map` that isn't included with our `SpearSim` executable.
  - `--vk-icd-filenames` only has an effect on Linux, and is used to force the Vulkan runtime to load a vendor-specific GPU driver. Our `run_executable.py` script will set the `VK_ICD_FILENAMES` environment variable to whatever is passed into `--vk-icd-filenames`. This argument may or may not be necessary, depending on your specific hardware setup. If you have already set the `VK_ICD_FILENAMES` environment variable before invoking `run_executable.py`, you do not need to specify `--vk-icd-filenames`. If you have an NVIDIA GPU, you probably need to specify `--vk-icd-filenames /usr/share/vulkan/icd.d/nvidia_icd.json`.

## Programmatically interact with SPEAR via Python

We provide several example applications that demonstrate how to programmatically interact with SPEAR via Python. In order to run our example applications, you will need to follow the steps in our [Running our Example Applications](running_our_example_applications.md) tutorial.
