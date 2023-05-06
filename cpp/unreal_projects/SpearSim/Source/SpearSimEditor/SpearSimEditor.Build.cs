//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;
using UnrealBuildTool;

public class SpearSimEditor : ModuleRules
{
    public SpearSimEditor(ReadOnlyTargetRules Target) : base(Target)
    {
        Console.WriteLine("[SPEAR | SpearSimEditor.Build.cs] SpearSimEditor::SpearSimEditor");
        Console.WriteLine("[SPEAR | SpearSimEditor.Build.cs] Target type: " + Target.Type);

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

        // Required for:
        //     ... > CoreUtils/Std.h    > boost/tokenizer.hpp > ... > boost/exception/exception.h
        //     ... > CoreUtils/Rpclib.h > rpc/msgpack.hpp     > ... > rpc/msgpack/predef/other/endian.h
        bEnableUndefinedIdentifierWarnings = false;

        PublicDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject", "CoreUtils", "Engine", "InputCore", "SpearSim", "UnrealEd"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
