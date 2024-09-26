//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool;

public class UrdfRobot : SpModuleRules
{
    public UrdfRobot(ReadOnlyTargetRules readOnlyTargetRules) : base(readOnlyTargetRules)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {"SpComponents", "SpCore"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
