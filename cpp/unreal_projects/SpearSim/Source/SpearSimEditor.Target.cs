//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;
using System.IO;
using System.Runtime.CompilerServices;
using UnrealBuildTool;

public class SpearSimEditorTarget : CommonTargetRulesTarget
{
    public SpearSimEditorTarget(TargetInfo target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        // Added to projects by default in UE 5.2.
        Type = TargetType.Editor;

        // We do include SpearSimEditor here, because the SpearSimEditor module needs to extend Unreal's UnrealEdEngine class, which is only
        // available in editor builds.
        ExtraModuleNames.AddRange(new string[] {"SpearSim", "SpearSimEditor"});
    }
}
