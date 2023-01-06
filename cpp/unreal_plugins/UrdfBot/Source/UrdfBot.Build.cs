//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool;
using System;
using System.Diagnostics;
using System.IO;

public class UrdfBot : ModuleRules
{
    public UrdfBot(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "CoreUtils", "Engine", "InputCore", "XmlParser" });
        PrivateDependencyModuleNames.AddRange(new string[] { "Slate" });

        //
        // Eigen
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "libeigen"));

        //
        // rbdl
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "include"));
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "include"));
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "Release", "rbdl.lib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "rbdl.a"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "rbdl.a"));
        }
        else
        {
            throw new Exception("Unexpected: Target.Platform == " + Target.Platform);
        }
    }
}
