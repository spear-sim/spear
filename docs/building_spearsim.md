# Building `SpearSim`

## Assumptions

We will assume for simplicity that you are developing on macOS, although most of these steps map straightforwardly across platforms. We will also assume that you're using Anaconda Python to manage your Python environment, and you have CMake installed. We will assume that you have cloned this entire repository including all submodules, and that you have installed the `spear` Python package installed, as described in our top-level [README](http://github.com/isl-org/spear). All `cd` commands in this tutorial are specified relative to the top-level repository directory.

## Install the Unreal Engine

We recommend installing the Unreal Engine version 4.26 via the Epic Games Launcher, rather than building it from source. You may need to disconnect from your VPN or proxy server when running the Epic Games Launcher. When you install the Unreal Engine, make sure you select _Editor symbols for debugging_ from the list of optional components.

If you're building on Linux, you will need to build the Unreal Engine from source. See [this tutorial](https://docs.unrealengine.com/4.26/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/) for details.

## Install XCode

In order to build the `SpearSim` project on macOS, you need to install a specific version of XCode that matches your Unreal Engine version. For Unreal Engine version 4.26, we have verified that XCode 13.0 behaves as expected. See [this tutorial](https://github.com/botman99/ue4-xcode-vscode-mac) for details.

## Build third-party C++ libraries

Our `SpearSim` project requires you to build several third-party C++ libraries. We provide a command-line tool for this purpose. You can adjust the `--num_parallel_jobs` argument for your system.

```console
cd tools
python build_third_party_libs.py --num_parallel_jobs 8
```

## Create symbolic links

Our `SpearSim` project requires you to create several symbolic links. We provide a command-line tool for this purpose.

If you're building on Windows, you will need to run this tool with administrator privileges.

```console
cd tools
python create_symbolic_links.py
```

## Generate a config file

Our `SpearSim` project assumes that all of its required parameters are declared in a configuration file. We usually launch our projects via the `spear` Python package, which takes care of generating the appropriate configuration file automatically. However, a valid configuration file is also required when building our projects, and we must generate this configuration file explicitly before attempting to build. To generate a configuration file, run the following command-line tool.

```console
cd tools
python generate_config.py
```

## Build the `SpearSim` executable

We build the `SpearSim` executable as follows.

```console
# build, cook, stage, package, archive
path/to/UE_4.26/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project=path/to/spear/cpp/unreal_projects/SpearSim/SpearSim.uproject -build -cook -stage -package -archive -pak -targetPlatform=Mac -target=SpearSim -clientconfig=Development -archivedirectory=path/to/spear/cpp/unreal_projects/SpearSim/Standalone-Development
```

This step will build a standalone executable at the following path. This executable is ready to use with any of our example applications.

```
cpp/unreal_projects/SpearSim/Standalone-Development/MacNoEditor/SpearSim.app
```

If you're running on Windows, you need to use the executable that is built at the following path.

```
cpp/unreal_projects/SpearSim/Standalone-Development/WindowsNoEditor/SpearSim/Binaries/Win64/SpearSim-Win64-Shipping-Cmd.exe
```

This executable ensures that `stdout` output is routed correctly to the Windows terminal.

### Helpful command-line options

- You can replace `-build` with `-skipbuild`, `-cook` with `-skipcook`, and `-stage -package -archive` with `-skipstage -skippackage -skiparchive`. After doing a complete `-build -cook -stage -package -archive`, you only need to `-cook` if you have edited the `.uproject` in the Unreal Editor, and you only need to `-stage -package -archive` if you want to update the standalone executable in `-archivedirectory`
- If you specify `-skipcook`, you can also specify `-nocompileeditor`, which saves time by not building a special executable that is only required when cooking.
- If you specify `-skipstage -skippackage -skiparchive`, you don't need to specify `-archivedirectory`.
- You can replace `Development` with `Shipping` to build a more optimized executable.
- You can specify `-clean` to do a clean build.
- You can specify `-verbose`, `-UbtArgs="-verbose"`, and `-UbtArgs="-VeryVerbose"` to see additional build details (e.g., the exact command-line arguments that Unreal uses when invoking the underlying compiler).
