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
        Console.WriteLine("[SPEAR | OpenBot.Build.cs] OpenBot::OpenBot");

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

        PublicDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject", "CoreUtils", "Engine", "PhysX", "PhysXVehicleLib", "PhysXVehicles"});
        PrivateDependencyModuleNames.AddRange(new string[] {});

        // TODO: This code needs to be wrapped in an #ifdef block because the function Directory.ResolveLinkTarget(...)
        // is not defined for the older C# standard library that ships with UE 4.26. Once we are ready to migrate to UE
        // 5.2, we can remove the #ifdef.

        #if UE_52
            // Resolve the top-level module directory and the ThirdParty directory, taking care to follow symlinks.
            // The top-level module directory can be a symlink or not, and the ThirdParty directory can be a symlink
            // or not. This is required to work around a bug that was introduced in UE 5.2.
            string topLevelModuleDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", ".."));
            FileSystemInfo topLevelModuleDirInfo = Directory.ResolveLinkTarget(topLevelModuleDir, true);
            topLevelModuleDir = (topLevelModuleDirInfo != null) ? topLevelModuleDirInfo.FullName : topLevelModuleDir;
            Console.WriteLine("[SPEAR | OpenBot.Build.cs] Resolved top-level module directory: " + topLevelModuleDir);

            string thirdPartyDir = Path.GetFullPath(Path.Combine(topLevelModuleDir, "ThirdParty"));
            FileSystemInfo thirdPartyDirInfo = Directory.ResolveLinkTarget(thirdPartyDir, true);
            thirdPartyDir = (thirdPartyDirInfo != null) ? thirdPartyDirInfo.FullName : thirdPartyDir;
            Console.WriteLine("[SPEAR | OpenBot.Build.cs] Resolved third-party directory: " + thirdPartyDir);
        #else
            string topLevelModuleDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", ".."));
            Console.WriteLine("[SPEAR | OpenBot.Build.cs] Resolved top-level module directory: " + topLevelModuleDir);

            string thirdPartyDir = Path.GetFullPath(Path.Combine(topLevelModuleDir, "ThirdParty"));
            Console.WriteLine("[SPEAR | OpenBot.Build.cs] Resolved third-party directory: " + thirdPartyDir);
        #endif

        //
        // Eigen
        //

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "libeigen", "BUILD", "Win64", "include", "eigen3")));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "libeigen", "BUILD", "Mac", "include", "eigen3")));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "libeigen", "BUILD", "Linux", "include", "eigen3")));
        } else {
            throw new Exception("[SPEAR | OpenBot.Build.cs] Unexpected target platform: " + Target.Platform);
        }
    }
}
