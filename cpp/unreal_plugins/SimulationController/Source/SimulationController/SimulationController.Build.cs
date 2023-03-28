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
        // easier debugging of compile errors, and strict enforcement of include-what-you-use.
        PCHUsage = ModuleRules.PCHUsageMode.Default;
        PrivatePCHHeaderFile = "";
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times.
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "CoreUtils", "Engine", "NavigationSystem", "OpenBot", "RenderCore", "RHI", "UrdfBot" });
        PrivateDependencyModuleNames.AddRange(new string[] {});

        // Our ASSERT macro throws exceptions, and so does our templated function Config::get(...),
        // because it depends on yaml-cpp, which throws exceptions. So we need to enable exceptions
        // everywhere. Note that boost::interprocess::mapped_region also throws exceptions, so we
        // would need to enable exceptions here even if we did not need them for our ASSERT macro
        // or Config::get(...).
        bEnableExceptions = true;

         // Required for boost::interprocess
        bEnableUndefinedIdentifierWarnings = false;

        //
        // Boost
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "..", "ThirdParty", "boost"));
    }
}
