# Building `SpearSim`

## Assumptions

We will assume for simplicity that you are developing on macOS, although most of these steps map straightforwardly across platforms. We will also assume that you're using Anaconda Python to manage your Python environment, and you have CMake installed. We will assume that you have cloned this entire repository including all submodules, and that you have installed the `spear` Python package installed, as described in our [Getting Started](getting_started.md) tutorial. All `cd` commands in this tutorial are specified relative to the top-level repository directory.

## Install the Unreal Engine

We recommend installing the Unreal Engine version 4.26 via the Epic Games Launcher, rather than building it from source. We recommend installing to a path that does not contain spaces. You may need to disconnect from your VPN or proxy server when running the Epic Games Launcher. When you install the Unreal Engine, make sure you select _Editor symbols for debugging_ from the list of optional components.

If you're building on Linux, you need to build the Unreal Engine from source. See [this tutorial](https://docs.unrealengine.com/4.26/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/) for details.

## Install XCode

In order to build the `SpearSim` project on macOS, you need to install a specific version of XCode that matches your Unreal Engine version. For Unreal Engine version 4.26, we have verified that XCode 13.0 behaves as expected. See [this tutorial](https://github.com/botman99/ue4-xcode-vscode-mac) for details.

## Install `clang` and `libc++`

If you're building on Linux, you need to install `clang` and `libc++`.

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

If you're building on Windows, you need to run this tool with administrator privileges.

```console
cd tools
python create_symbolic_links.py --unreal_engine_dir path/to/UE_4.26
```

The `--unreal_engine_dir` argument must point to the top-level directory where you installed (or built) the Unreal Engine. Depending on your platform, the default install location will be as follows. However, as noted above, we recommend installing the Unreal Engine to a path that doesn't contain spaces. If you're building on Linux, you must specify the path to the top-level `UnrealEngine` GitHub repository.

```
Windows: C:\Program Files\Epic Games\UE_4.26
macOS:   /Users/Shared/Epic Games/UE_4.26
Linux:   path/to/github/UnrealEngine
```

## Build the `SpearSim` executable

We build the `SpearSim` executable as follows.

```console
# build, cook, stage, package, archive
path/to/UE_4.26/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project=path/to/spear/cpp/unreal_projects/SpearSim/SpearSim.uproject -build -cook -stage -package -archive -pak -iterativecooking -targetPlatform=Mac -target=SpearSim -clientconfig=Development -archivedirectory=path/to/spear/cpp/unreal_projects/SpearSim/Standalone-Development
```

Depending on your platform, this step will build an executable at different paths.

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
