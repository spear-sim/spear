// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class PlayEnvironment : ModuleRules
{
    public PlayEnvironment(ReadOnlyTargetRules Target) : base(Target)
    {
        // Disable precompiled headers for faster builds, easier debugging, and stricter enforcement of "include what you use"
        PCHUsage = ModuleRules.PCHUsageMode.NoPCHs;
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster builds
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Enable exceptions because some of our third-party dependencies use them
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
