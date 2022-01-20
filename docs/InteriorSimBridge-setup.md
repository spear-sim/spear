# InterioSimBridge plugin Setup

- InteriorSimBridge plugin acts a bridge plugin between RobotSim and UnrealRL plugin.
- InteriorSimBridge plugin contains 'UBrain' implementations for different robots available in RobotSim plugin such as OpenBot.

## Setup UnrealRL plugin

- Follow steps outlined in `code/thirdparty/unreal-ai/README.md` to setup UnrealRL plugin.
- Add UnrealRL plugin available in `code/thirdparty/unreal-ai/unreal/Plugins/UnrealRL` to the Unreal Project.

## How to use InteriorSimBridge plugin

- To use InteriorSimBridge plugin, make sure both RobotSim and UnrealRL plugins are part of the InteriorSim Project.
- Depending on what experiment needs to be run, you can write a 'UBrain' implementation similar to `code/unreal_plugins/InteriorSimBridge/Source/InteriorSimBridge/SimpleVehicleBrain.cpp`.
- Make sure the new implementation is handled/included in 'InteriorSimBridgeManager.cpp''s `OnWorldInitializedActors()` and `OnActorSpawned()` functions.

## How to use UnrealAI library's python client

- Navigate to `code/thirdparty/unreal-ai/client/python` directory and follows steps as outlined in the README file.
- Use example code - `code/python_client/example_code.py` to write your own experiments.