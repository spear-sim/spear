# Running OpenBot within interiorsim as a standalone executable in a RL context: 

The following ReadMe file will guide you through the differents steps required to execute the OpenBot simulation within a photorealistic environment using a python client. It is here assumed that you are running Ubuntu 20.04.

## Setting up the interiorsim repo

### Install and build pre-requisite libraries

Start by installing the following libraries: 

```
sudo apt install clang
sudo apt install libc++-dev libc++abi-dev
sudo apt install libeigen3-dev
```

### Download and build RBDL

Download and build the Rigid Body Dynamics Library (RBDL). Rather than cloning RBDL from the official (repository)[https://github.com/rbdl/rbdl], consider the following (fork)[https://github.com/quentin-leboutet/rbdl] which solves a set of compile-time issues you may encounter with the main branch. Then build RBDL using the following commands: 

```
cd interiorsim/code/thirdparty
git clone https://github.com/rbdl/rbdl
cd rbdl
mkdir BUILD
cd BUILD
cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_STATIC=ON -D RBDL_BUILD_ADDON_URDFREADER=ON -D CMAKE_CXX_COMPILER="clang++" -D CMAKE_CXX_FLAGS="-fPIC -stdlib=libc++" ../
make
```

### Set symbolic links to the plugins 

Assuming you are in the *interiorsim* root:
```
cd interiorsim/code/unreal_projects/RobotProject
mkdir Plugin 
cd Plugin 
ln -s ../../../unreal_plugins/InteriorSimBridge/ .
ln -s ../../../unreal_plugins/RobotSim/ .
ln -s path/to/UnrealRL/ .
```

### Configure system paths

Rename `path/to/interiorsim/code/unreal_plugins/RobotSim/Source/RobotSim.Build.cs.example -> RobotSim.Build.cs` and modify the paths at the top of this file for your system.


### Build and run RobotProject as a standalone executable

The first time the project is being built, the following command should be used: 

```
# build cook and package (needs to be done the first time you build or when you modify an asset)
path/to/UE_4.26/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -nop4 -project=path/to/interiorsim/code/unreal_projects/RobotProject/RobotProject.uproject -build -cook -stage -package -archive -archivedirectory=path/to/interiorsim/code/unreal_projects/RobotProject/dist -targetplatform=Linux -target=RobotProject -nocompileeditor -nodebuginfo -serverconfig=Development -clientconfig=Development -package
```

In case the build and cooking process were already executed and no asset was edited, the cooking process can be skipped for the next build iterations:

```
# build, skip cook (allows saving compile time when no asset was modified)
path/to/UE_4.26/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project=path/to/interiorsim/code/unreal_projects/RobotProject/RobotProject.uproject -build -skipcook -skipstage -skiparchive -targetplatform=Linux -target=RobotProject -nocompileeditor -nodebuginfo -serverconfig=Development -clientconfig=Development -package
```

One can eventually test the obtained executable using the follwing command: 

```
cd path/to/interiorsim/code/unreal_projects/RobotProject/dist/LinuxNoEditor
cp -R path/to/interiorsim/code/unreal_plugins/RobotSim/setting .
./RobotProject.sh -WINDOWED -ResX=512 -ResY=512 
```
The robot should spawn in a simplistic environment. Now lets spawn the OpenBot in a more complex photorealistic environement... 

## Load a new scene in UE4

### Download a photorealistic interior environment

Download one of the available photorealistic interior environments (for instance 235553720):

```
cd code/unreal_projects/RobotProject/SceneManager
scene_manager.py -i 235553720 -v v2 -d true
```

### Edit the interior environment using the Unreal Editor 

Once downloaded, you should load your environment into the unreal editor. To do so, double-click on the project link ```unreal_projects/RobotProject/RobotProject.uproject```. The Unreal Editor should start and load a simple project with an empty environment. You should then load the new map using the content browser in the lower left corner of your screen:

<img src="docs/unreal_editor.png" width="100%" alt="Goal Tag" />

The map should take around 30 seconds to load the first time you open it: 

<img src="docs/load_map.png" width="100%" alt="Goal Tag" />

You should then be able to explore the environment you downloaded: 

<img src="docs/load_map.png" width="100%" alt="Goal Tag" />

Executing the RL code requires a "goal" actor towards which your agent should move. As the goal actor is not natively included in the maps, you should therefore add it manually. To do so, simply add an empty pawn to your freshly downloaded map using the "PlaceActor" menu on the left of your screen. The new pawn should be in a reachable location and -- more importantly -- should be labelled as a "goal". You can label your pawn using the "Actor" properties menu on the right of your screen: 

<img src="docs/goal.png" width="100%" alt="Goal Tag" />

Save your changes to the environment. Then **rebuild + cook** your project:

```
# cook and package (needs to be done the first time you build or when you modify an asset)
path/to/UE_4.26/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -nop4 -project=path/to/interiorsim/code/unreal_projects/RobotProject/RobotProject.uproject -build -cook -stage -archive -archivedirectory=path/to/interiorsim/code/unreal_projects/RobotProject/dist -targetplatform=Linux -target=RobotProject -nocompileeditor -nodebuginfo -serverconfig=Development -clientconfig=Development -package
```

You should now be ready to test the executable using the python client...

## Setting the python client in the unreal-ai repo

Under the old Unreal-AI conventions, the content of my *unreal-ai/client/python/unrealai/config.yaml* file was: 

```
# configuration file
---
env_config:
  server_ip: &server_ip ""
  server_port: &server_port "8000"
  timeout_value: 3600
  reconn_tries: 99999999
  enable_thread_sync: True
  enable_sync_mode: True
  delta_seconds: 0.1 
  binary_path: "/home/quentin/Desktop/ISL/interiorsim/code/unreal_projects/RobotProject/dist/LinuxNoEditor/RobotProject.sh"
  #binary_path: "/home/quentin/Desktop/ISL/unreal-ai/examples/InteriorEnvironment/dist/LinuxNoEditor/InteriorEnvironment.sh"
  #binary_path: "/home/quentin/Desktop/ISL/unreal-ai/examples/CityParkEnvironment/dist/LinuxNoEditor/CityParkEnvironment.sh"
  #binary_path: "/home/quentin/Desktop/ISL/unreal-ai/examples/PlayEnvironment/dist/LinuxNoEditor/PlayEnvironment.sh"
  #binary_path: "/home/quentin/Desktop/ISL/UnrealEngine-4.26.2-release/Engine/Binaries/Linux/UE4Editor"
  launch_params: 
    - "/Game/Maps/Map_235553720" # Allows to start the packaged environment while automatically loading the desired map.
    - "-game"
    - "-rlport="
    - *server_port
    - "-rlip="
    - *server_ip
    - "-windowed"
    - "-resx=512"
    - "-resy=512"
    - "-novsync"
    - "-rlseed=123"
    - "-NoSound"
    - "-RobotSimSettingPath=/home/quentin/Desktop/ISL/interiorsim/code/unreal_projects/RobotProject/dist/LinuxNoEditor/setting/settings.json" # Allows to avoid "settings.json not found" error at simulation start
    #- "-RenderOffscreen"
```
Under the new conventions, my config.yaml file looks like:

```
UNREALAI:
# launch unreal engine instance via different modes
# "standalone_executable", or "uproject", or "running_instance"
# "standalone_executable" - launches a packaged executable of an Unreal Engine project
# "uproject" - launches an uproject with Unreal Engine editor executable
# "running_instance" - connects to an already running instance of Unreal Engine
  LAUNCH_MODE: "standalone_executable"

# if launch_mode is "standalone_exectuable", specify this path to an unreal engine executable
  STANDALONE_EXECUTABLE: "/home/quentin/Desktop/ISL/interiorsim/code/unreal_projects/RobotProject/dist/LinuxNoEditor/RobotProject.sh"

# if launch_mode is "uproject", specify this path to your unreal engine editor executable
  UNREAL_EDITOR_EXECUTABLE: "/home/quentin/Desktop/ISL/UnrealEngine-4.26.2-release/Engine/Binaries/Linux/UE4Editor"

# if launch_mode is "uproject", specify this path to your unreal engine project file
  UPROJECT: "/home/quentin/Desktop/ISL/interiorsim/code/unreal_projects/RobotProject/RobotProject.uproject"

# path to a temp dir where this library has read and write access
  TEMP_DIR: "/home/quentin/Desktop/ISL/unreal-ai/client"

# True if unreal engine instance should be launched in headless mode
# Note: This flag is unused if your launch_mode is "running_instance"
  RENDER_OFFSCREEN: False

# Allows to avoid "settings.json not found" error at simulation start
  ROBOTSIM_SETTING_PATH: "/home/quentin/Desktop/ISL/interiorsim/code/unreal_projects/RobotProject/dist/LinuxNoEditor/setting/settings.json"

# Path to the map you want to load.
# Allows to start the packaged environment while automatically loading the desired map.
  MAP_PATH: "/Game/Maps/Map_235553720"

# currently we require to sleep for sometime after unreal engine exectuable launch
# might vary from system to system or use-case
# should remove this in future as it's an hack and instead handle launch gracefully
  SLEEP_TIME_DURING_LAUNCH_SECONDS: 10
```

## Running the RL.py script to launch the OpenBot simulation

Assuming that you have a virtual env named *interiorsim-env*, run the RL script providing the path to your **config.yaml** parameter file:

```
cd interiorsim/code/experiments/RL_script
conda activate interiorsim-env
python RL.py --user_config_file="path/to/config.yaml"
```
