//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool; // ReadOnlyTargetRules

public class SpUnrealTypesEditor : SpModuleRules
{
    public SpUnrealTypesEditor(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {"SpCore", "UnrealEd"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
