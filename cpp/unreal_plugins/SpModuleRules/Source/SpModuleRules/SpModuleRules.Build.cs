//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;                          // Console, Exception
using System.IO;                       // Directory, FileSystemInfo, Path 
using System.Runtime.CompilerServices; // CallerFilePath, CallerLineNumber, CallerMemberName
using UnrealBuildTool;                 // ModuleRules, ReadOnlyTargetRules

public class SpModuleRules : ModuleRules
{
    public SpModuleRules(ReadOnlyTargetRules readOnlyTargetRules) : base(readOnlyTargetRules)
    {
        SP_LOG_CURRENT_FUNCTION();

        // Disable precompiled headers (in our code but not Unreal code) for faster builds,
        // easier debugging of compile errors, and strict enforcement of include-what-you-use.
        PCHUsage = ModuleRules.PCHUsageMode.Default;
        PrivatePCHHeaderFile = "";
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times.
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Our SP_ASSERT macro throws exceptions, yaml-cpp (used by Config) throws exceptions,
        // and boost::interprocess::mapped_region (used by camera sensors) throws exceptions.
        // So we need to enable exceptions everywhere.
        bEnableExceptions = true;

        // Required for:
        //     ... > SpCore/Std.h    > boost/tokenizer.hpp > ... > boost/exception/exception.h
        //     ... > SpCore/Rpclib.h > rpc/msgpack.hpp     > ... > rpc/msgpack/predef/other/endian.h
        bEnableUndefinedIdentifierWarnings = false;

        PublicDependencyModuleNames.AddRange(new string[] {
            "Chaos", "ChaosVehiclesCore", "Core", "CoreUObject", "Engine", "EngineSettings", "InputCore", "Json", "JsonUtilities", "NavigationSystem",
            "PhysicsCore", "RenderCore", "RHI", "XmlParser"});
        PrivateDependencyModuleNames.AddRange(new string[] {});

        // Resolve the top-level module directory and the ThirdParty directory, taking care to follow symlinks.
        // The top-level module directory can be a symlink or not, and the ThirdParty directory can be a symlink
        // or not. This is required to work around a bug that was introduced in UE 5.2.
        string topLevelModuleDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", ".."));
        FileSystemInfo topLevelModuleDirInfo = Directory.ResolveLinkTarget(topLevelModuleDir, true);
        topLevelModuleDir = (topLevelModuleDirInfo != null) ? topLevelModuleDirInfo.FullName : topLevelModuleDir;

        string thirdPartyDir = Path.GetFullPath(Path.Combine(topLevelModuleDir, "ThirdParty"));
        FileSystemInfo thirdPartyDirInfo = Directory.ResolveLinkTarget(thirdPartyDir, true);
        thirdPartyDir = (thirdPartyDirInfo != null) ? thirdPartyDirInfo.FullName : thirdPartyDir;

        //
        // Boost
        //

        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost")));

        if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_unit_test_framework.lib")));
        } else if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_unit_test_framework.a")));
        } else if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_unit_test_framework.a")));
        } else {
            throw new Exception(SP_LOG_GET_PREFIX() + "Unexpected target platform: " + readOnlyTargetRules.Platform);
        }

        //
        // rpclib
        //

        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "include")));

        if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "BUILD", "Win64", "Release", "rpc.lib")));
        } else if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "BUILD", "Mac", "librpc.a")));
        } else if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "rpclib", "BUILD", "Linux", "librpc.a")));
        } else {
            throw new Exception(SP_LOG_GET_PREFIX() + "Unexpected target platform: " + readOnlyTargetRules.Platform);
        }

        //
        // yaml-cpp
        //

        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "include")));

        if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Win64) {
            PublicDefinitions.Add("YAML_CPP_STATIC_DEFINE");
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "BUILD", "Win64", "Release", "yaml-cpp.lib")));
        } else if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "BUILD", "Mac", "libyaml-cpp.a")));
        } else if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "yaml-cpp", "BUILD", "Linux", "libyaml-cpp.a")));
        } else {
            throw new Exception(SP_LOG_GET_PREFIX() + "Unexpected target platform: " + readOnlyTargetRules.Platform);
        }
    }

    protected void SP_LOG(string message, [CallerFilePath] string filePath="", [CallerLineNumber] int lineNumber=0)
    {
        Console.WriteLine(GetPrefix(filePath, lineNumber) + message);
    }

    protected void SP_LOG_CURRENT_FUNCTION([CallerFilePath] string filePath="", [CallerLineNumber] int lineNumber=0, [CallerMemberName] string memberName="")
    {
        Console.WriteLine(GetPrefix(filePath, lineNumber) + GetCurrentFunctionExpanded(memberName));
    }

    protected string SP_LOG_GET_PREFIX([CallerFilePath] string filePath="", [CallerLineNumber] int lineNumber=0)
    {
        return GetPrefix(filePath, lineNumber);
    }

    private string GetPrefix(string filePath, int lineNumber)
    {
        return "[SPEAR | " + GetCurrentFileAbbreviated(filePath) + ":" + lineNumber.ToString("D4") + "] ";
    }

    private string GetCurrentFileAbbreviated(string filePath)
    {
        return Path.GetFileName(filePath);
    }

    private string GetCurrentFunctionExpanded(string memberName)
    {
        string sep = memberName.StartsWith(".") ? "" : ".";
        return this.GetType() + sep + memberName;
    }
}
