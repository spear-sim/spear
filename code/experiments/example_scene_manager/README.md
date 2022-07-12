# ExampleSceneProject

example project for VirtualWorldManager showcases.

### How to Use

1. create symlink for required plugins: CoreUtils and VirtualWorldManager
    ```buildoutcfg
    ln -s /home/xichen/intel/robotproject/code/unreal_plugins/CoreUtils /home/xichen/intel/robotproject/code/experiments/example_scene_manager/ExampleSceneProject/Plugins/CoreUtils

    ln -s /home/xichen/intel/robotproject/code/unreal_plugins/VirtualWorldManager /home/xichen/intel/ robotproject/code/experiments/example_scene_manager/ExampleSceneProject/Plugins/VirtualWorldManager
    ```

2. build project

   ```buildoutcfg
   /home/xichen/program/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh ExampleSceneProjectEditor Linux Development \
   /home/xichen/intel/robotproject/code/experiments/example_scene_manager/ExampleSceneProject/ExampleSceneProject.uproject
   ```
3. run in editor

   ```buildoutcfg
   /home/xichen/program/UnrealEngine/Engine/Binaries/Linux/UE4Editor /home/xichen/intel/robotproject/code/experiments/example_scene_manager/ExampleSceneProject/ExampleSceneProject.uproject
   ```

4. world set GameMode as `AExampleSceneProjectGameModeBase`

5. play around with key `SpaceBar`,``1`, `2`, `3` 


### How to Use DoorManager
1. load an IS scene level otherwise it does not work.
2. initialize level door info  
   ```
   UVWDoorManager::initLevelDoorInfo(GetWorld());
   ```
3. open or close doors in current level. Note that some doors might not be stable after movement
   ```buildoutcfg
   UVWDoorManager::moveAllDoor(door_stat_);
   ```