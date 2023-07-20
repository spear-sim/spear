# Building `SpearSim`

## Assumptions

We will assume that you are developing on a version of Windows, macOS, or Linux that is compatible with Unreal Engine 5.2. We will also assume that you're using Anaconda Python to manage your Python environment, and that you have CMake installed. We will assume that you have cloned this entire repository including all submodules, and that you have installed the `spear` Python package, as described in our [Getting Started](getting_started.md) tutorial. All `cd` commands in this tutorial are specified relative to the top-level repository directory.

## Install the Unreal Engine

We recommend installing the Unreal Engine version 5.2 via the Epic Games Launcher, rather than building it from source. We recommend installing to a path that does not contain spaces. You may need to disconnect from your VPN or proxy server when running the Epic Games Launcher. When you install the Unreal Engine, make sure you select _Editor symbols for debugging_ from the list of optional components.

If you're developing on Linux, you will need to download the Unreal Engine from [here](https://www.unrealengine.com/en-US/linux).

## Install the appropriate compiler for your platform

For each platform, you will need to install a specific compiler version that matches Unreal Engine 5.2.

```
Windows: Visual Studio 2022
macOS:   XCode 14.3
Linux:   clang and libc++
```

If you're developing on Linux, you can install clang and libc++ as follows.

```console
sudo apt install libc++-dev libc++abi-dev clang
```

## Build third-party C++ libraries

Our `SpearSim` project requires you to build several third-party C++ libraries. We provide a command-line tool for this purpose.

```console
cd tools
python build_third_party_libs.py
```

This command-line tool accepts an optional `--num_parallel_jobs` argument. This argument can be used to specify the number of parallel jobs that `cmake` should use when building third-party libraries.

## Create symbolic links

Our `SpearSim` project requires you to create several symbolic links. We provide a command-line tool for this purpose.

If you're developing on Windows, you will need to run this tool with administrator privileges.

```console
cd tools
python create_symlinks.py
```

## Copy Unreal Engine starter content

Our `SpearSim` project requires you to explicitly copy some starter content from your Unreal Engine installation to the project folder.

```console
cd tools
python copy_starter_content.py --unreal_engine_dir path/to/UE_5.2
```

The `--unreal_engine_dir` argument must point to the top-level directory where you installed the Unreal Engine. Depending on your platform, the default install location will be as follows. However, as noted above, we recommend installing the Unreal Engine to a path that doesn't contain spaces. If you're developing on Linux, you must specify the path to the top-level directory where you unzipped the `Linux_Unreal_Engine_5.2.0.zip` you downloaded earlier.

```
Windows: C:\Program Files\Epic Games\UE_5.2
macOS:   /Users/Shared/Epic Games/UE_5.2
Linux:   path/to/Linux_Unreal_Engine_5.2.0/
```

## Build the `SpearSim` executable

We build the `SpearSim` executable as follows.

```console
# build, cook, stage, package, archive
path/to/UE_5.2/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project=path/to/spear/cpp/unreal_projects/SpearSim/SpearSim.uproject -build -cook -stage -package -archive -pak -iterativecooking -targetPlatform=Mac -target=SpearSim -clientconfig=Development -archivedirectory=path/to/spear/cpp/unreal_projects/SpearSim/Standalone-Development
```

Depending on your platform, you will need to specify `-targetPlatform` as `Win64`, `Mac`, or `Linux`. You will obtain an executable at one of the following locations.

```
Windows: cpp/unreal_projects/SpearSim/Standalone-Development/WindowsNoEditor/SpearSim/Binaries/Win64/SpearSim-Cmd.exe
macOS:   cpp/unreal_projects/SpearSim/Standalone-Development/MacNoEditor/SpearSim.app
Linux:   cpp/unreal_projects/SpearSim/Standalone-Development/LinuxNoEditor/SpearSim.sh
```

### Helpful command-line options

- You can replace `-build` with `-skipbuild`, `-cook` with `-skipcook`, and `-stage -package -archive` with `-skipstage -skippackage -skiparchive`. After doing a complete `-build -cook -stage -package -archive`, you only need to `-cook` if you have edited the project in the Unreal Editor, and you only need to `-stage -package -archive` if you want to update the standalone executable in `-archivedirectory`
- If you specify `-skipcook`, you can also specify `-nocompileeditor`, which saves time by not building a special executable that is only required when cooking.
- If you specify `-skipstage -skippackage -skiparchive`, you don't need to specify `-archivedirectory`.
- If you only want to propagate changes in `cpp/unreal_projects/SpearSim/Config` to the executable in `-archivedirectory`, you can specify `-skipbuild -skipcook`.
- You can replace `Development` with `Shipping` to build a more optimized executable.
- You can specify `-clean` to do a clean build.
- You can specify `-verbose`, `-UbtArgs="-verbose"`, and `-UbtArgs="-VeryVerbose"` to see additional build details (e.g., the exact command-line arguments that Unreal uses when invoking the underlying compiler).
