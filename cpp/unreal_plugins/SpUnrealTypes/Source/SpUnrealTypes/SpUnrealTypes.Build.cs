//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;          // Exception
using UnrealBuildTool; // ReadOnlyTargetRules

public class SpUnrealTypes : SpModuleRules
{
    public SpUnrealTypes(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        //
        // As a matter of convenience, it is possible to place most Unreal module dependencies in SpModuleRules
        // without needing to make any changes to our uplugin files. Plugin modules are different. If we list
        // a plugin module in SpModuleRules or SpModuleRulesEditor, then we must also list the plugin module
        // in all of our uplugin files. To avoid this unnecessary clutter, we only list plugin modules in Build.cs
        // files belonging to modules where the plugins are actually used.
        //

        PublicDependencyModuleNames.AddRange(new string[] {"MovieRenderPipelineCore", "MovieRenderPipelineRenderPasses", "PCG", "SpCore"});
        PrivateDependencyModuleNames.AddRange(new string[] {});

        if (target.Platform == UnrealTargetPlatform.Win64) {
            PublicDependencyModuleNames.AddRange(new string[] {"D3D11RHI", "D3D12RHI", "VulkanRHI"});
            AddEngineThirdPartyPrivateStaticDependencies(target, "DX11");
            AddEngineThirdPartyPrivateStaticDependencies(target, "DX12");
            AddEngineThirdPartyPrivateStaticDependencies(target, "Vulkan");
        } else if (target.Platform == UnrealTargetPlatform.Mac) {
            // We don't query Metal memory stats, mimicking UE 5.8's no-op ::RHIGetMemoryStats() on Metal, so no
            // Metal-specific module dependencies are required.
        } else if (target.Platform == UnrealTargetPlatform.Linux) {
            PublicDependencyModuleNames.AddRange(new string[] {"VulkanRHI"});
            AddEngineThirdPartyPrivateStaticDependencies(target, "Vulkan");
        } else {
            throw new Exception(SP_LOG_GET_PREFIX() + "Unexpected target platform: " + target.Platform);
        }
    }
}
