//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool; // ReadOnlyTargetRules

public class SpServicesEditor : SpModuleRulesEditor
{
    public SpServicesEditor(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {"SpCore", "SpCoreEditor", "SpServices"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
