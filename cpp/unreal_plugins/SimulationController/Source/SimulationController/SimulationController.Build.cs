//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;
using System.IO;
using UnrealBuildTool;

public class SimulationController : ModuleRules
{
    public SimulationController(ReadOnlyTargetRules Target) : base(Target)
    {
        // Disable precompiled headers (in our code but not Unreal code) for faster builds,
        // easier debugging of compile errors, and strict enforcement of include-what-you-use
        PCHUsage = ModuleRules.PCHUsageMode.Default;
        PrivatePCHHeaderFile = "";
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Enable exceptions because some of our third-party dependencies use them
        bEnableExceptions = true;

        bEnableUndefinedIdentifierWarnings = false;

        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "CoreUtils", "Engine", "NavigationSystem", "OpenBot", "RenderCore", "RHI" });
        PrivateDependencyModuleNames.AddRange(new string[] {});

        //
        // asio
        //

        PublicDefinitions.Add("ASIO_NO_TYPEID");
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "asio", "asio", "include"));

        //
        // boost
        //

        PublicIncludePaths.Add("/Users/mroberts/code/github/boost/libs/assert/include");
        PublicIncludePaths.Add("/Users/mroberts/code/github/boost/libs/config/include");
        PublicIncludePaths.Add("/Users/mroberts/code/github/boost/libs/core/include");
        PublicIncludePaths.Add("/Users/mroberts/code/github/boost/libs/interprocess/include");
        PublicIncludePaths.Add("/Users/mroberts/code/github/boost/libs/intrusive/include");
        PublicIncludePaths.Add("/Users/mroberts/code/github/boost/libs/move/include");
        PublicIncludePaths.Add("/Users/mroberts/code/github/boost/libs/static_assert/include");

        //
        // rpclib
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "rpclib", "include"));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "rpclib", "BUILD", "Win64", "Release", "rpc.lib"));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "rpclib", "BUILD", "Mac", "librpc.a"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "rpclib", "BUILD", "Linux", "librpc.a"));
        } else {
            throw new Exception("Target.Platform == " + Target.Platform);
        }
    }
}
