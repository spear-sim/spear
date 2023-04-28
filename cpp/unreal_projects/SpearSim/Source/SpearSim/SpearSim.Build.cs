//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;
using UnrealBuildTool;

public class SpearSim : ModuleRules
{
    public SpearSim(ReadOnlyTargetRules Target) : base(Target)
    {
        Console.WriteLine("[SPEAR | SpearSim.Build.cs] SpearSim::SpearSim");

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

        // This is required for the usage of boost/tokenizer.h, and boost/predef.h. If not, boost throws the following exceptions.
        // C:\github\spear\third_party\boost\boost\exception\exception.hpp(22): error C4668:
        //      '__GNUC__' is not defined as a preprocessor macro, replacing with '0' for '#if/#elif'
        // C:\github\spear\third_party\boost\boost\exception\exception.hpp(22): error C4668:
        //      '__GNUC_MINOR__' is not defined as a preprocessor macro, replacing with '0' for '#if/#elif'
        bEnableUndefinedIdentifierWarnings = false;

        PublicDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject", "CoreUtils", "Engine", "InputCore"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
