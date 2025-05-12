//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool; // ReadOnlyTargetRules

public class SpComponents : SpModuleRules
{
    public SpComponents(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {"ChaosVehicles", "MovieRenderPipelineCore", "MovieRenderPipelineRenderPasses", "SpCore"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
