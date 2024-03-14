//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool;

public class SpearSimEditor : SpModuleRules
{
    public SpearSimEditor(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();
        
        PublicDependencyModuleNames.AddRange(new string[] {"SpCore", "UnrealEd", "UrdfRobot", "Vehicle"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
