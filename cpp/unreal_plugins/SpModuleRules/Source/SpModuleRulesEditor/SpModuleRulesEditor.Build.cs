//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool; // ReadOnlyTargetRules

public class SpModuleRulesEditor : SpModuleRules
{
    public SpModuleRulesEditor(ReadOnlyTargetRules target) : base(target)
    {
        PublicDependencyModuleNames.AddRange(new string[] {"EditorSubsystem", "UnrealEd"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
