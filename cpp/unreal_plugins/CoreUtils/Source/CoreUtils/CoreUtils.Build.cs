//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;
using System.IO;
using UnrealBuildTool;

public class CoreUtils : ModuleRules
{
    public CoreUtils(ReadOnlyTargetRules Target) : base(Target)
    {
        // Disable precompiled headers (in our code but not Unreal code) for faster builds,
        // easier debugging of compile errors, and strict enforcement of include-what-you-use.
        PCHUsage = ModuleRules.PCHUsageMode.Default;
        PrivatePCHHeaderFile = "";
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times.
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Our ASSERT macro throws exceptions, and so does our templated function Config::get(...),
        // because it depends on yaml-cpp, which throws exceptions. So we need to enable exceptions
        // everywhere.
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new string[] {});

        //
        // Boost
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "boost"));

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
            throw new Exception("[SPEAR | CoreUtils.Build.cs] Target.Platform == " + Target.Platform);
        }

        //
        // yaml-cpp
        //

        bEnableExceptions = true;
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "yaml-cpp", "include"));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicDefinitions.Add("YAML_CPP_STATIC_DEFINE");
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "yaml-cpp", "BUILD", "Win64", "Release", "yaml-cpp.lib"));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "yaml-cpp", "BUILD", "Mac", "libyaml-cpp.a"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "yaml-cpp", "BUILD", "Linux", "libyaml-cpp.a"));
        } else {
            throw new Exception("[SPEAR | CoreUtils.Build.cs] Unexpected: Target.Platform == " + Target.Platform);
        }
    }
}
