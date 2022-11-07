// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class InteriorEnvironmentTarget : TargetRules
{
    public InteriorEnvironmentTarget( TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange( new string[] { "InteriorEnvironment" } );

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            bBuildAdditionalConsoleApp = true;
        }
    }
}
