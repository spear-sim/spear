//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>
// Copyright Epic Games, Inc. All Rights Reserved.
//

using UnrealBuildTool;

public class SpearSimTarget : TargetRules
{
    public SpearSimTarget( TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange( new string[] {"SpearSim"});

        // On Windows, we need to build an additional app so that calls to UE_Log and writes to std::cout are visible on the command-line
        if (Target.Platform == UnrealTargetPlatform.Win64) {
            bBuildAdditionalConsoleApp = true;
        }
    }
}
