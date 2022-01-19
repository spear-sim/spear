// Copyright Epic Games, Inc. All Rights Reserved.

using System;
using System.IO;
using System.Collections.Generic;
using UnrealBuildTool;

public class UnrealRL : ModuleRules
{
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    public UnrealRL(ReadOnlyTargetRules Target) : base(Target)
    {
        // TODO: Check what this does / why we need it.
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        // This helps with build errors due to try/throw in asio caused by disabled exceptions (e.g. C4530 warning on windows)
        bEnableExceptions = true;

        // Change this to 1, if debug info has to be logged
        PublicDefinitions.Add("UNREALRL_DEBUG=0");

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Engine"
            }
            );

        // Add rpclib.
        PublicDefinitions.Add("MSGPACK_PP_VARIADICS_MSVC=0");
        PublicIncludePaths.Add(Path.Combine(ModulePath, "..", "ThirdParty", "rpclib", "include"));
        String rpcpath = Path.Combine(ModulePath, "..", "ThirdParty", "rpclib", "lib");
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            rpcpath = Path.Combine(rpcpath, "rpc.lib");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac)
        {
            rpcpath = Path.Combine(rpcpath, "librpc.a");
        }
        PublicAdditionalLibraries.Add(rpcpath);

        // Add asio.
        PublicDefinitions.Add("ASIO_NO_TYPEID");
        PublicIncludePaths.Add(Path.Combine(ModulePath, "..", "ThirdParty", "asio"));
    }
}
