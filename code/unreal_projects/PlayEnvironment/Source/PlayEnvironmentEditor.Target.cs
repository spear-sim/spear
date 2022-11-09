using UnrealBuildTool;

public class PlayEnvironmentEditorTarget : TargetRules
{
    public PlayEnvironmentEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange(new string[] { "PlayEnvironment" });

        // Disabling precompiled headers doesn't seem to be supported for Editor builds
        // bUsePCHFiles = false;
        // bUseSharedPCHs = false;
        // bUseUnityBuild = false;

        // On Windows, we need to build an additional app so that calls to UE_Log and writes to std::cout are visible on the command-line
        if (Target.Platform == UnrealTargetPlatform.Win64) {
            bBuildAdditionalConsoleApp = true;
        }
    }
}
