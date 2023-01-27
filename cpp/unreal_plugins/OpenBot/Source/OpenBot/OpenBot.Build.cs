//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;
using System.IO;
using UnrealBuildTool;

public class OpenBot : ModuleRules
{
    public OpenBot(ReadOnlyTargetRules Target) : base(Target)
    {
        // Disable precompiled headers (in our code but not Unreal code) for faster builds,
        // easier debugging of compile errors, and strict enforcement of include-what-you-use
        PCHUsage = ModuleRules.PCHUsageMode.Default;
        PrivatePCHHeaderFile = "";
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "CoreUtils", "Engine", "PhysX", "PhysXVehicleLib", "PhysXVehicles" });
        PrivateDependencyModuleNames.AddRange(new string[] {});

        //
        // Eigen
        //

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "libeigen", "BUILD", "Win64", "include", "eigen3"));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "libeigen", "BUILD", "Mac", "include", "eigen3"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "libeigen", "BUILD", "Linux", "include", "eigen3"));
        } else {
            throw new Exception("Unexpected: Target.Platform == " + Target.Platform);
        }
    }
}
