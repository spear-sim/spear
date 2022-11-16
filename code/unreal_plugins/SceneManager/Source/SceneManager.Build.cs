using UnrealBuildTool;

public class SceneManager : ModuleRules
{
    public SceneManager(ReadOnlyTargetRules Target) : base(Target)
    {
        // Disable precompiled headers (in our code but not Unreal code) for faster builds, easier debugging of compile errors, and strict enforcement of include-what-you-use
        PCHUsage = ModuleRules.PCHUsageMode.Default;
        PrivatePCHHeaderFile = "";
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster build times
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Enable exceptions because some of our third-party dependencies use them
        bEnableExceptions = true;
        
        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "CoreUtils", "Engine", "PhysicsCore" });
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
