# InSim and UnrealRL plugin

- UnreaRL plugin is used to perform Reinforcement learning experiments.
- InSim plugin is used as a bridge plugin between RobotSim and UnrealRL plugins.
- InSim plugin combines information from RobotSim and UnrealRL plugin, for experiments with InteriorSim environments.
- InSim plugin contains 'UBrain' implementations for different robots available in RobotSim plugin such as OpenBot.

## How to setup UnrealRL plugin

- Make sure you have cloned this repo and it's submodules.
- Run `utils/build_external_libs.cmd` (on windows) or `utils/build_external_libs.sh` (on unix) to build the required libraries for UnrealRL plugin.
- Add UnrealRL plugin available in `code/unreal_plugins/UnrealRL` to the Unreal Project.

## How to use InSim plugin

- To use InSim plugin, make sure both RobotSim and UnrealRL plugins are part of the InteriorSim Project.
- Depending on what experiment needs to be run, you can write a 'UBrain' implementation similar to `code/unreal_plugins/InSim/Source/InSim/SimpleVehicleBrain.cpp`.
- Make sure the new implementation is handled/included in 'InSimManager.cpp''s `OnWorldInitializedActors()` and `OnActorSpawned()` functions.

## How to use unrealai python client

- Navigate to `code/python_client` directory and follows steps as outlined in `code/python_client/README.md`.
- Use example code - `code/python_client/example_code.py` to write your own experiments.