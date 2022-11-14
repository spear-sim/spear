using UnrealBuildTool;

public class RobotProjectTarget : TargetRules
{
    public RobotProjectTarget( TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange(new string[] { "RobotProject" });

        // Disable precompiled headers for faster builds, easier debugging of compile errors, and stricter enforcement of include-what-you-use
        bUsePCHFiles = false;
        bUseSharedPCHs = false;
        bUseUnityBuild = false;

        // On Windows, we need to build an additional app so that calls to UE_Log and writes to std::cout are visible on the command-line
        if (Target.Platform == UnrealTargetPlatform.Win64) {
            bBuildAdditionalConsoleApp = true;
        }
    }
}
