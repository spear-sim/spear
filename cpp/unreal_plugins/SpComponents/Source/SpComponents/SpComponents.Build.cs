//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool;

public class SpComponents : SpModuleRules
{
    public SpComponents(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] { "ChaosVehicles", "SpCore" });
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
