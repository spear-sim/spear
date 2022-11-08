// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class PlayEnvironmentTarget : TargetRules
{
    public PlayEnvironmentTarget( TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange( new string[] { "PlayEnvironment" } );

        // Disable precompiled headers for faster builds, easier debugging, and stricter enforcement of "include what you use"
        bUsePCHFiles = false;
        bUseSharedPCHs = false;
        bUseUnityBuild = false;
    }
}
