//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool; // TargetInfo

public class SpearSimTarget : SpTargetRulesTarget
{
    public SpearSimTarget(TargetInfo targetInfo) : base(targetInfo)
    {
        // Added to projects by default in UE 5.5.
        Type = TargetType.Game;
    }
}
