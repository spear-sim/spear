//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool; // TargetInfo

public class SpearSimEditorTarget : SpTargetRulesTarget
{
    public SpearSimEditorTarget(TargetInfo targetInfo) : base(targetInfo)
    {
        SP_LOG_CURRENT_FUNCTION();

        // Added to projects by default in UE 5.7.
        Type = TargetType.Editor;
    }
}
