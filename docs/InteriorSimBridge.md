# InterioSimBridge plugin
- InteriorSimBridge plugin acts a bridge between RobotSim and UnrealRL plugin.
- UnrealRL plugin is part of the UnrealAI library. For more information on UnrealAI, go [here](../code/thirdparty/unreal-ai/README.md).

## Setup InteriorSim project to work with InteriorSimBridge and UnrealRL plugin

### Setup UnrealRL plugin
- Follow steps outlined in [UnrealRL readme](../code/thirdparty/unreal-ai/README.md) to setup UnrealRL plugin.
- Add [UnrealRL plugin](../code/thirdparty/unreal-ai/unreal/Plugins/UnrealRL) to the Unreal project.

### Setup InteriorSimBridge plugin
- Add [InteriorSimBridge plugin](../code/unreal_plugins/InteriorSimBridge) to the Unreal project.

### How to use InteriorSimBridge plugin

- To use InteriorSimBridge plugin, make sure both RobotSim and UnrealRL plugins are part of the InteriorSim Project.
- Depending on the experiment that needs to be run, you can write a 'UBrain' implementation similar to [SimpleVehicleBrain.cpp](../code/unreal_plugins/InteriorSimBridge/Source/InteriorSimBridge/SimpleVehicleBrain.cpp).
- Make sure the new implementation is handled/included in InteriorSimBridge.cpp file in `WorldInitializedActorsEventHandler()` and `ActorSpawnedEventHandler()` functions.

## How to use UnrealAI library's python client
- Navigate to [UnrealAI's python client](../code/thirdparty/unreal-ai/client/python) directory and follows steps as outlined in the README file to setup the python client.
- Use [example code](../code/python_client/example_code.py) to write your own experiments.
