//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool; // ReadOnlyTargetRules

public class SpCoreEditor : SpModuleRules
{
    public SpCoreEditor(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {"LevelEditor", "SpCore", "UnrealEd"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
