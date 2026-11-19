//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;                          // Console
using System.IO;                       // Path
using System.Runtime.CompilerServices; // CallerFilePath, CallerLineNumber, CallerMemberName
using UnrealBuildTool;                 // ModuleRules, ReadOnlyTargetRules

public class SpModuleRules : ModuleRules
{
    public SpModuleRules(ReadOnlyTargetRules readOnlyTargetRules) : base(readOnlyTargetRules)
    {
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
        //     ... > SpCore/Std.h        > boost/tokenizer.hpp > ... > boost/exception/exception.h
        //     ... > SpServices/Rpclib.h > rpc/msgpack.hpp     > ... > rpc/msgpack/predef/other/endian.h
        UndefinedIdentifierWarningLevel = WarningLevel.Warning;

        PublicDependencyModuleNames.AddRange(new string[] {
            "AssetRegistry", "Chaos", "Core", "CoreUObject", "Engine", "EngineSettings", "InputCore", "Json", "JsonUtilities", "Landscape",
            "LevelSequence", "NavigationSystem", "PhysicsCore", "RenderCore", "Renderer", "RHI", "RHICore", "Slate"});
        PrivateDependencyModuleNames.AddRange(new string[] {});

        // Required for our RHI interface
        if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Win64) {
            PublicDependencyModuleNames.AddRange(new string[] {"D3D11RHI", "D3D12RHI", "VulkanRHI"});
            AddEngineThirdPartyPrivateStaticDependencies(readOnlyTargetRules, "DX11", "DX12", "Vulkan");
        } else if (readOnlyTargetRules.Platform == UnrealTargetPlatform.Linux) {
            PublicDependencyModuleNames.Add("VulkanRHI");
            AddEngineThirdPartyPrivateStaticDependencies(readOnlyTargetRules, "Vulkan");
        }

        // Needed to expose a private header in Renderer
        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(readOnlyTargetRules.RelativeEnginePath, "Source", "Runtime", "Renderer", "Private")));

        // MessageLog is a Developer-category module, and is therefore not available in Shipping builds.
        if (readOnlyTargetRules.bBuildDeveloperTools) {
            PublicDependencyModuleNames.Add("MessageLog");
            PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(readOnlyTargetRules.RelativeEnginePath, "Source", "Developer", "MessageLog", "Private")));
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
