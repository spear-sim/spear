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
        Console.WriteLine("[SPEAR | CoreUtils.Build.cs] CoreUtils::CoreUtils");

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

        PublicDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject", "Engine"});
        PrivateDependencyModuleNames.AddRange(new string[] {});

        // Resolve the top-level module directory and the ThirdParty directory, taking care to follow symlinks.
        // The top-level module directory can be a symlink or not, and the ThirdParty directory can be a symlink
        // or not. This is required to work around a bug that was introduced in UE 5.2.
        string topLevelModuleDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", ".."));
        FileSystemInfo topLevelModuleDirInfo = Directory.ResolveLinkTarget(topLevelModuleDir, true);
        topLevelModuleDir = (topLevelModuleDirInfo != null) ? topLevelModuleDirInfo.FullName : topLevelModuleDir;
        Console.WriteLine("[SPEAR | CoreUtils.Build.cs] Resolved top-level module directory: " + topLevelModuleDir);

        string thirdPartyDir = Path.GetFullPath(Path.Combine(topLevelModuleDir, "ThirdParty"));
        FileSystemInfo thirdPartyDirInfo = Directory.ResolveLinkTarget(thirdPartyDir, true);
        thirdPartyDir = (thirdPartyDirInfo != null) ? thirdPartyDirInfo.FullName : thirdPartyDir;
        Console.WriteLine("[SPEAR | CoreUtils.Build.cs] Resolved third-party directory: " + thirdPartyDir);

        //
        // Boost
        //

        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost")));

        //
        // rpclib
        //

        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "include")));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "BUILD", "Win64", "Release", "rpc.lib")));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "BUILD", "Mac", "librpc.a")));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "BUILD", "Linux", "librpc.a")));
        } else {
            throw new Exception("[SPEAR | CoreUtils.Build.cs] Unexpected target platform: " + Target.Platform);
        }

        //
        // yaml-cpp
        //

        bEnableExceptions = true;
        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "include")));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicDefinitions.Add("YAML_CPP_STATIC_DEFINE");
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "BUILD", "Win64", "Release", "yaml-cpp.lib")));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "BUILD", "Mac", "libyaml-cpp.a")));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "BUILD", "Linux", "libyaml-cpp.a")));
        } else {
            throw new Exception("[SPEAR | CoreUtils.Build.cs] Unexpected target platform: " + Target.Platform);
        }
    }
}
