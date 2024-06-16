# Building `SpearSim`

## Assumptions

We will assume that you are developing on a version of Windows, macOS, or Linux that is compatible with Unreal Engine 5.2. We will also assume that you're using Anaconda Python to manage your Python environment, and that you have CMake installed. We will assume that you have cloned this entire repository including all submodules, and that you have installed the `spear` Python package, as described in our [Getting Started](getting_started.md) tutorial.

## Install the Unreal Engine

We recommend installing the Unreal Engine version 5.2 via the Epic Games Launcher, rather than building it from source. We recommend installing to a path that does not contain spaces. You may need to disconnect from your VPN or proxy server when running the Epic Games Launcher. When you install the Unreal Engine, make sure to select _Editor symbols for debugging_ from the list of optional components.

If you're developing on Linux, you will need to download the Unreal Engine from [here](https://www.unrealengine.com/en-US/linux).

## Install an appropriate compiler

For each platform, you will need to install a specific compiler that is compatible with Unreal Engine 5.2. We have verified that the following compilers behave as expected.

```
Windows: Visual Studio 2022
macOS:   XCode 14.3
Linux:   clang 15.0.1
```

If you're developing on Windows, make sure to select _Desktop development with C++_ from the _Workloads_ menu when installing Visual Studio.

If you're developing on Linux, you can install `clang` and `libc++` as follows.

```console
sudo apt install libc++-dev libc++abi-dev clang
```

## Configure your terminal

If you're developing on Windows, you will need to run our build steps in a terminal that meets the following requirements.

- Your terminal must be able to access Anaconda Python, `cmake`, and the Visual Studio command-line tools.
- You will need to run our build steps from within the Anaconda environment where the `spear` package is installed.
- You will need to one of run our build steps with administrator privileges (noted below), because we need to create various symbolic links within your local copy of this repository to work around limitations of the Unreal build system.

There are multiple possible ways to satisfy these requirements. One possibility is to use the _Developer PowerShell for VS 2022_ profile in the Windows Terminal application to run our build steps. This profile will be installed to the Terminal application when you install Visual Studio. You can configure this profile to run with administrator privileges in the profile settings. Additionally, you can configure PowerShell to access your Anaconda environment by opening an Anaconda PowerShell prompt with administrator privileges (located in your Start menu after you install Anaconda) and executing the following commands.

```console
# This step will add a block of PowerShell code to C:\Users\username\Documents\WindowsPowerShell\profile.ps1
# to make your Anaconda installation visible to PowerShell.
conda init powershell

# When running as administrator, PowerShell will automatically load C:\Users\username\Documents\WindowsPowerShell\profile.ps1,
# but this step will allow it to load profile.ps1 even when running as a normal user, so Anaconda will always be visible.
Set-ExecutionPolicy -Scope LocalMachine -ExecutionPolicy Bypass
```

After executing these commands, you can use the _Developer PowerShell for VS 2022_ profile in the Windows Terminal application, and it will be able to access Anaconda Python and the Visual Studio command-line tools.

## Build third-party C++ libraries

Our `SpearSim` project requires you to build several third-party C++ libraries. We provide a command-line tool for this purpose.

```console
python tools/build_third_party_libs.py
```

This command-line tool accepts an optional `--num_parallel_jobs` argument. This argument can be used to specify the number of parallel jobs that `cmake` should use when building third-party libraries.

## Create symbolic links

Our `SpearSim` project requires you to create several symbolic links. We provide a command-line tool for this purpose.

If you're developing on Windows, you will need to run this command with administrator privileges.

```console
python tools/create_symlinks.py
```

## Copy starter content

Our `SpearSim` project requires you to explicitly copy some starter content from your Unreal Engine installation to the project directory. We provide a command-line tool for this purpose.

```console
python tools/copy_starter_content.py --unreal_engine_dir path/to/UE_5.2
```

The `--unreal_engine_dir` argument must point to the top-level directory where you installed the Unreal Engine. Depending on your platform, the default install location will be as follows. However, as noted above, we recommend installing the Unreal Engine to a path that doesn't contain spaces. If you're developing on Linux, you must specify the path to the top-level directory where you unzipped the `Linux_Unreal_Engine_5.2.0.zip` you downloaded earlier.

```
Windows: C:\Program Files\Epic Games\UE_5.2
macOS:   /Users/Shared/Epic Games/UE_5.2
Linux:   path/to/Linux_Unreal_Engine_5.2.0
```

## Build the `SpearSim` executable

We are now ready to build the `SpearSim` executable as follows.

```console
# build, cook, stage, package, archive
cd tools
python tools/run_uat.py --unreal_engine_dir path/to/UE_5.2 --build_config Development -build -cook -stage -package -archive -pak -iterativecooking
```

Note that our `run_uat.py` tool is a thin wrapper around Unreal's [RunUAT](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/Deployment/BuildOperations) tool. Our tool consumes `--unreal_engine_dir` and `--build_config`, provides `RunUAT` with sensible default values for a few commonly used arguments, and otherwise forwards all arguments directly to `RunUAT`. This step will generate an executable at the following locations.

```
Windows: cpp/unreal_projects/SpearSim/Standalone-Development/Windows/SpearSim/Binaries/Win64/SpearSim-Cmd.exe
macOS:   cpp/unreal_projects/SpearSim/Standalone-Development/Mac/SpearSim.app
Linux:   cpp/unreal_projects/SpearSim/Standalone-Development/Linux/SpearSim.sh
```

### Helpful command-line options

- After you have done a complete `-build -cook -stage -package -archive` once, you can replace `-build` with `-skipbuild`, `-cook` with `-skipcook`, and `-stage -package -archive` with `-skipstage -skippackage -skiparchive`, depending on what you're doing. You only need to `-cook` if you have edited the project in the Unreal Editor, and you only need to `-stage -package -archive` if you want to update the standalone executable in `-archivedirectory`.
- If you specify `-skipcook`, you can also specify `-nocompileeditor`, which saves time by not building a second executable that is only required when cooking.
- If you specify `-skipstage -skippackage -skiparchive`, you don't need to specify `-archivedirectory`.
- If you only want to propagate changes in `cpp/unreal_projects/SpearSim/Config` to the executable in `-archivedirectory`, you can specify `-skipbuild -skipcook`.
- You can replace `Development` with `Shipping` to build a more optimized executable.
- You can specify `-specifiedarchitecture=arm64+x86_64` to build a universal binary on macOS.
- You can specify `-clean` to do a clean build.
- You can specify `-verbose`, `-UbtArgs="-verbose"`, and `-UbtArgs="-VeryVerbose"` to see additional build details (e.g., the exact command-line arguments that Unreal uses when invoking the underlying compiler).
