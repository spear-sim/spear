# Virtual World Manager

Virtual World Manager is an Unreal Plugin providing API for Virtual World loading, and modification such as asset
layout, lighting, etc at run time.

### Loading Virtual World for standalone executables

Use SceneManager to download .pak files for VirtualWorld then move the .pak files to `<path_to_standalone>/RobotProject/Content/Paks`. Then start the simulator and .pak files will be
mounted.
* `VWLevelmanager::GetAllMapsInPak`: find all available level names from mounted .pak.

### more coming