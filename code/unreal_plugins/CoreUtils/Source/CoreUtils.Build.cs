// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System;
using System.Diagnostics;
using System.IO;

public class CoreUtils : ModuleRules
{
    public CoreUtils(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core" });
        PrivateDependencyModuleNames.AddRange(new string[] {});

        //
        // yaml-cpp
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "yaml-cpp", "include"));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicDefinitions.Add("YAML_CPP_STATIC_DEFINE");
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "yaml-cpp", "build", "Release", "yaml-cpp.lib"));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "yaml-cpp", "build", "libyaml-cpp.a"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "yaml-cpp", "build", "libyaml-cpp.a"));
        } else {
            throw new Exception("Unexpected: Target.Platform == " + Target.Platform);
        }
    }
}
