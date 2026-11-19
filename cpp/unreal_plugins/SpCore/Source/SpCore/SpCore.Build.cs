//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;          // Exception
using System.IO;       // Path
using UnrealBuildTool; // ReadOnlyTargetRules, UnrealTargetPlatform

public class SpCore : SpModuleRules
{
    public SpCore(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {});
        PrivateDependencyModuleNames.AddRange(new string[] {});

        // We only add our third-party libraries here, because all of our other modules depend on SpCore and
        // therefore get these include paths and libraries transitively. If multiple modules added the same
        // libraries, they would appear multiple times on the linker command-line.

        string thirdPartyDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", "..", "..", "..", "..", "third_party"));

        //
        // boost
        //

        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost")));

        // We need to link against unit_test_framework because SpCore/Boost.h uses boost::debug::under_debugger
        // from <boost/test/debug.hpp>, even though we don't use Boost's unit testing functionality.
        if (target.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_unit_test_framework-vc143-mt-x64-1_90.lib")));
        } else if (target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_unit_test_framework.a")));
        } else if (target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.GetFullPath(Path.Combine(thirdPartyDir, "boost", "stage", "lib", "libboost_unit_test_framework.a")));
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
}
