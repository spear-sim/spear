using UnrealBuildTool;

public class PlayEnvironment : ModuleRules
{
    public PlayEnvironment(ReadOnlyTargetRules Target) : base(Target)
    {
        // We want to disable precompiled headers for faster builds, easier debugging of compile errors,
        // and stricter enforcement of include-what-you-use. But it seems as though Editor builds must
        // be built with precompiled headers, and Editor builds are required during the cooking process.
        if (Target.Type == TargetType.Editor) {
            PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        } else {
            PCHUsage = ModuleRules.PCHUsageMode.NoPCHs;
            bUseUnity = false;
        }

        // Turn off code optimization except in shipping builds for faster build times
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Enable exceptions because some of our third-party dependencies use them
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
