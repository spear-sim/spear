using UnrealBuildTool;
using System.IO;

public class OpenBot : ModuleRules
{
    public OpenBot(ReadOnlyTargetRules Target) : base(Target)
    {
        // Disable precompiled headers for faster builds, easier debugging of compile errors, and stricter enforcement of "include what you use"
        PCHUsage = ModuleRules.PCHUsageMode.NoPCHs;
        bUseUnity = false;

        // Turn off code optimization except in shipping builds for faster builds
        OptimizeCode = ModuleRules.CodeOptimization.InShippingBuildsOnly;

        // Enable exceptions because some of our third-party dependencies use them
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "CoreUtils", "Engine", "PhysX", "PhysXVehicleLib", "PhysXVehicles" });
        PrivateDependencyModuleNames.AddRange(new string[] {});

        //
        // Eigen
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "libeigen"));
    }
}
