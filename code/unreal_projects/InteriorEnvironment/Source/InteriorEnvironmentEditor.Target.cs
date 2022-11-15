using UnrealBuildTool;

public class InteriorEnvironmentEditorTarget : TargetRules
{
    public InteriorEnvironmentEditorTarget( TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange(new string[] { "InteriorEnvironment" });

        // On Windows, we need to build an additional app so that calls to UE_Log and writes to std::cout are visible on the command-line
        if (Target.Platform == UnrealTargetPlatform.Win64) {
            bBuildAdditionalConsoleApp = true;
        }
    }
}
