using System;
using System.IO;
using UnrealBuildTool;

public class SimulationController : ModuleRules
{
    public SimulationController(ReadOnlyTargetRules Target) : base(Target)
    {
        // Disable precompiled headers (in our code but not Unreal code) for faster builds, easier debugging of compile errors, and strict enforcement of include-what-you-use
        PCHUsage = ModuleRules.PCHUsageMode.Default;
        PrivatePCHHeaderFile = "";
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Enable exceptions because some of our third-party dependencies use them
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "CoreUtils", "Engine", "NavigationSystem", "OpenBot", "RenderCore", "RHI", "SceneManager" });
        PrivateDependencyModuleNames.AddRange(new string[] {});

        //
        // asio
        //

        PublicDefinitions.Add("ASIO_NO_TYPEID");
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "asio", "asio", "include"));

        //
        // rpclib
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rpclib", "include"));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rpclib", "build", "Release", "rpc.lib"));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rpclib", "build", "librpc.a"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rpclib", "build", "librpc.a"));
        } else {
            throw new Exception("Target.Platform == " + Target.Platform);
        }
    }
}
