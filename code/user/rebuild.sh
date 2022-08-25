#!/bin/bash

rm -r unreal_projects/RobotProject/Binaries 
rm -r unreal_projects/RobotProject/Intermediate
rm -r unreal_projects/RobotProject/Build
rm -r unreal_projects/RobotProject/DerivedDataCache
rm -r unreal_projects/RobotProject/Saved
rm -r unreal_projects/RobotProject/Plugins/RobotSim/Binaries 
rm -r unreal_projects/RobotProject/Plugins/RobotSim/Intermediate

/home/qleboute/Documents/Git/UnrealEngine-4.26.2-release/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project=/home/qleboute/Documents/Git/interiorsim/code/unreal_projects/RobotProject/RobotProject.uproject -build -skipcook -stage -archive -archivedirectory=/home/qleboute/Documents/Git/interiorsim/code/unreal_projects/RobotProject/dist -targetplatform=Linux -target=RobotProject -nodebuginfo -package -clientconfig=Development -pak

