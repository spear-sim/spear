// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System;
using System.Diagnostics;
using System.IO;

public class SimulationController : ModuleRules
{
    public SimulationController(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "CoreUtils", "Engine", "NavigationSystem", "OpenBot", "RenderCore", "RHI", "SceneManager" });
        PrivateDependencyModuleNames.AddRange(new string[] {});

        //
        // asio
        //

        PublicDefinitions.Add("ASIO_NO_TYPEID");
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "asio", "asio", "include"));

        //
        // rpclib
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rpclib", "include"));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rpclib", "build", "Release", "rpc.lib"));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rpclib", "build", "librpc.a"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rpclib", "build", "librpc.a"));
        } else {
            throw new Exception("Unexpected: Target.Platform == " + Target.Platform);
        }
    }
}
