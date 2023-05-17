//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;
using System.IO;
using System.Runtime.CompilerServices;
using UnrealBuildTool;

public class CommonModuleRules : ModuleRules
{
    public CommonModuleRules(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        // Disable precompiled headers (in our code but not Unreal code) for faster builds,
        // easier debugging of compile errors, and strict enforcement of include-what-you-use.
        PCHUsage = ModuleRules.PCHUsageMode.Default;
        PrivatePCHHeaderFile = "";
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times.
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Our ASSERT macro throws exceptions, yaml-cpp (used by Config) throws exceptions,
        // and boost::interprocess::mapped_region (used by CameraSensor) throws exceptions.
        // So we need to enable exceptions everywhere.
        bEnableExceptions = true;

        // Required for:
        //     ... > CoreUtils/Std.h    > boost/tokenizer.hpp > ... > boost/exception/exception.h
        //     ... > CoreUtils/Rpclib.h > rpc/msgpack.hpp     > ... > rpc/msgpack/predef/other/endian.h
        bEnableUndefinedIdentifierWarnings = false;

        PublicDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject", "Engine", "InputCore", "NavigationSystem", "RenderCore", "RHI"});
        PrivateDependencyModuleNames.AddRange(new string[] {});

        // Resolve the top-level module directory and the ThirdParty directory, taking care to follow symlinks.
        // The top-level module directory can be a symlink or not, and the ThirdParty directory can be a symlink
        // or not. This is required to work around a bug that was introduced in UE 5.2.
        string topLevelModuleDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", ".."));
        FileSystemInfo topLevelModuleDirInfo = Directory.ResolveLinkTarget(topLevelModuleDir, true);
        topLevelModuleDir = (topLevelModuleDirInfo != null) ? topLevelModuleDirInfo.FullName : topLevelModuleDir;
        SP_LOG("Resolved top-level module directory: " + topLevelModuleDir);

        string thirdPartyDir = Path.GetFullPath(Path.Combine(topLevelModuleDir, "ThirdParty"));
        FileSystemInfo thirdPartyDirInfo = Directory.ResolveLinkTarget(thirdPartyDir, true);
        thirdPartyDir = (thirdPartyDirInfo != null) ? thirdPartyDirInfo.FullName : thirdPartyDir;
        SP_LOG("Resolved third-party directory: " + thirdPartyDir);

        //
        // Boost
        //

        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost")));

        //
        // Eigen
        //

        if (target.Platform == UnrealTargetPlatform.Win64) {
            PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "libeigen", "BUILD", "Win64", "include", "eigen3")));
        } else if (target.Platform == UnrealTargetPlatform.Mac) {
            PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "libeigen", "BUILD", "Mac", "include", "eigen3")));
        } else if (target.Platform == UnrealTargetPlatform.Linux) {
            PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "libeigen", "BUILD", "Linux", "include", "eigen3")));
        } else {
            throw new Exception(SP_LOG_GET_PREFIX() + "Unexpected target platform: " + target.Platform);
        }

        //
        // rpclib
        //

        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "include")));

        if (target.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "BUILD", "Win64", "Release", "rpc.lib")));
        } else if (target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "BUILD", "Mac", "librpc.a")));
        } else if (target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "BUILD", "Linux", "librpc.a")));
        } else {
            throw new Exception(SP_LOG_GET_PREFIX() + "Unexpected target platform: " + target.Platform);
        }

        //
        // yaml-cpp
        //

        bEnableExceptions = true;
        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "include")));

        if (target.Platform == UnrealTargetPlatform.Win64) {
            PublicDefinitions.Add("YAML_CPP_STATIC_DEFINE");
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "BUILD", "Win64", "Release", "yaml-cpp.lib")));
        } else if (target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "BUILD", "Mac", "libyaml-cpp.a")));
        } else if (target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "BUILD", "Linux", "libyaml-cpp.a")));
        } else {
            throw new Exception(SP_LOG_GET_PREFIX() + "Unexpected target platform: " + target.Platform);
        }
    }

    protected void SP_LOG(string message, [CallerFilePath] string filePath="")
    {
        Console.WriteLine(GetPrefix(filePath) + message);
    }

    protected void SP_LOG_CURRENT_FUNCTION([CallerFilePath] string filePath="", [CallerMemberName] string memberName="")
    {
        Console.WriteLine(GetPrefix(filePath) + GetCurrentFunctionExpanded(memberName));
    }

    protected string SP_LOG_GET_PREFIX([CallerFilePath] string filePath="")
    {
        return GetPrefix(filePath);
    }

    private string GetPrefix(string filePath)
    {
        return "[SPEAR | " + GetCurrentFileAbbreviated(filePath) + "] ";
    }

    private string GetCurrentFileAbbreviated(string filePath)
    {
        return Path.GetFileName(filePath);
    }

    private string GetCurrentFunctionExpanded(string memberName)
    {
        return this.GetType() + "." + ((memberName == ".ctor") ? this.GetType() : memberName);
    }
}
