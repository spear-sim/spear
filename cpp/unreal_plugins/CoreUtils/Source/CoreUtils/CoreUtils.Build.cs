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
        // Disable precompiled headers (in our code but not Unreal code) for faster builds, easier debugging of compile errors, and strict enforcement of include-what-you-use
        PCHUsage = ModuleRules.PCHUsageMode.Default;
        PrivatePCHHeaderFile = "";
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Enable exceptions because some of our third-party dependencies use them
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new string[] {});

        //
        // yaml-cpp
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "yaml-cpp", "include"));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicDefinitions.Add("YAML_CPP_STATIC_DEFINE");
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "yaml-cpp", "BUILD", "Win64", "Release", "yaml-cpp.lib"));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "yaml-cpp", "BUILD", "Mac", "libyaml-cpp.a"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "yaml-cpp", "BUILD", "Linux", "libyaml-cpp.a"));
        } else {
            throw new Exception("Unexpected: Target.Platform == " + Target.Platform);
        }
    }
}
