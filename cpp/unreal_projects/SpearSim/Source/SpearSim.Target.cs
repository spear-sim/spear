//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool;

public class SpearSimTarget : SpTargetRulesTarget
{
    public SpearSimTarget(TargetInfo target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        // Added to projects by default in UE 5.2.
        Type = TargetType.Game;
    }
}
