//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool;

public class SimulationController : CommonModuleRules
{
    public SimulationController(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {"CoreUtils"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
