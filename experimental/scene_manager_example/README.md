# SceneManagerExample

example project for SceneManager showcases.

### Set Up SceneManagerExample

1. create symlink for required plugins: `CoreUtils` and `SceneManager`

 ```buildoutcfg
ln -s /home/xichen/intel/robotproject/code/unreal_plugins/CoreUtils /home/xichen/intel/robotproject/code/experiments/scene_manager_example/SceneManagerExample/Plugins/CoreUtils
ln -s /home/xichen/intel/robotproject/code/unreal_plugins/SceneManager /home/xichen/intel/robotproject/code/experiments/scene_manager_example/SceneManagerExample/Plugins/SceneManager
```

2. copy RobotSim Content to `Content` folder

3. build SceneManager.

```buildoutcfg
/home/xichen/program/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh SceneManagerExampleEditor Linux Development /home/xichen/intel/robotproject/code/experiments/scene_manager_example/SceneManagerExample/SceneManagerExample.uproject
```

4. Run Editor

```buildoutcfg
/home/xichen/program/UnrealEngine/Engine/Binaries/Linux/UE4Editor /home/xichen/intel/robotproject/code/experiments/scene_manager_example/SceneManagerExample/SceneManagerExample.uproject
```

4. change the map Default Pawn to ExamplePawn.

5. Start simulation, press `SpaceBar`,`1`, `2`, `3` to check out SceneManager functionalities.

6. Build standalone executable:

```buildoutcfg
/home/xichen/program/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project=/home/xichen/intel/robotproject/code/experiments/scene_manager_example/SceneManagerExample/SceneManagerExample.uproject -build -cook -stage -package -archive -targetPlatform=Linux -pak -target=SceneManagerExample -clientconfig=Development -archivedirectory=/home/xichen/intel/robotproject/dist
```

### PhysicalManager
Provide API to change physical property such as friction and density. 