//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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

        // Disable precompiled headers entirely because they somehow force full rebuilds in UE 5.5.
        // Additionally, we prefer to avoid precompiled headers for easier debugging of compile errors, and
        // stricter enforcement of include-what-you-use.
        PCHUsage = PCHUsageMode.NoPCHs;

        // Disable unity builds for easier debugging of compile errors, and stricter enforcement of
        // include-what-you-use.
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times.
        OptimizeCode = CodeOptimization.InShippingBuildsOnly;

        // Our error handling code throws exceptions, our SP_ASSERT macro throws exceptions, yaml-cpp (used
        // by Config) throws exceptions, and boost::interprocess::mapped_region (used by SharedMemoryRegion)
        // throws exceptions. So we enable exceptions everywhere.
        bEnableExceptions = true;

        // Required for:
        //     ... > SpCore/Std.h    > boost/tokenizer.hpp > ... > boost/exception/exception.h
        //     ... > SpCore/Rpclib.h > rpc/msgpack.hpp     > ... > rpc/msgpack/predef/other/endian.h
        UndefinedIdentifierWarningLevel = WarningLevel.Warning;

        PublicDependencyModuleNames.AddRange(new string[] {
            "Chaos", "ChaosVehiclesCore", "Core", "CoreUObject", "Engine", "EngineSettings", "InputCore", "Json", "JsonUtilities", "LevelSequence",
            "NavigationSystem", "PhysicsCore", "RenderCore", "RHI", "Slate", "XmlParser"});
        PrivateDependencyModuleNames.AddRange(new string[] {});

        string thirdPartyDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", "..", "..", "..", "..", "third_party"));

        //
        // Boost
        //

        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost")));

        if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_filesystem-vc143-mt-x64-1_81.lib")));
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_unit_test_framework-vc143-mt-x64-1_81.lib")));
        } else if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_filesystem.a")));
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_unit_test_framework.a")));
        } else if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_filesystem.a")));
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
