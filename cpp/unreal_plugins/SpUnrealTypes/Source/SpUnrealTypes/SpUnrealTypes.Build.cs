//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

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

        PublicDependencyModuleNames.AddRange(new string[] {"MovieRenderPipelineCore", "MovieRenderPipelineRenderPasses", "SpCore"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
