// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class InteriorEnvironmentEditorTarget : TargetRules
{
    public InteriorEnvironmentEditorTarget( TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange( new string[] { "InteriorEnvironment" } );

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            bBuildAdditionalConsoleApp = true;
        }
    }
}
