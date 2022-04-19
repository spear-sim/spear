// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System;
using System.IO;

public class RobotSim : ModuleRules
{
    public RobotSim(ReadOnlyTargetRules Target) : base(Target)
    {
        OptimizeCode = CodeOptimization.Never;
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        bEnableExceptions = true;

        PublicIncludePaths.AddRange(new string[] {} );
        PrivateIncludePaths.AddRange(new string[] {} );
        
        PublicDependencyModuleNames.AddRange(new string[] {
            "APEX", "Core", "CoreUObject", "Engine", "Foliage", "InputCore", "ImageWrapper", "NavigationSystem", "PhysicsCore", "PhysX",
            "PhysXVehicles", "PhysXVehicleLib", "ProceduralMeshComponent", "Landscape", "RenderCore", "RHI", "Slate", "SlateCore", "UMG", "XmlParser", "VirtualWorldManager"
        } );
        PrivateDependencyModuleNames.AddRange(new string[] {} );

        //
        // Eigen
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "libeigen"));
        
        //
        // RBDL
        //

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "include"));
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "include"));
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty"));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "rbdl.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "addons", "urdfreader", "rbdl_urdfreader.lib"));
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "librbdl.a"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "addons", "urdfreader", "librbdl_urdfreader.a"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "librbdl.a"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "rbdl", "build", "addons", "urdfreader", "librbdl_urdfreader.a"));
        } else {
            throw new Exception("Unexpected: Target.Platform == " + Target.Platform);
        }

        //
        // VHACD
        //

        PublicIncludePaths.Add(Path.Combine(Target.UEThirdPartySourceDirectory, "VHACD", "public"));

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            if (Target.Configuration == UnrealTargetConfiguration.Debug && Target.bDebugBuildsActuallyUseDebugCRT) {
                PublicAdditionalLibraries.Add(Path.Combine(Target.UEThirdPartySourceDirectory, "VHACD", "lib", "Win64", "VS" + Target.WindowsPlatform.GetVisualStudioCompilerVersionName(), "VHACDd.lib"));
            } else {
                PublicAdditionalLibraries.Add(Path.Combine(Target.UEThirdPartySourceDirectory, "VHACD", "lib", "Win64", "VS" + Target.WindowsPlatform.GetVisualStudioCompilerVersionName(), "VHACD.lib"));
            }
        } else if (Target.Platform == UnrealTargetPlatform.Mac) {
            if (Target.Configuration == UnrealTargetConfiguration.Debug && Target.bDebugBuildsActuallyUseDebugCRT) {
                PublicAdditionalLibraries.Add(Path.Combine(Target.UEThirdPartySourceDirectory, "VHACD", "Lib", "Mac", "libVHACD_LIBd.a"));
            } else {
                PublicAdditionalLibraries.Add(Path.Combine(Target.UEThirdPartySourceDirectory, "VHACD", "Lib", "Mac", "libVHACD_LIB.a"));
            }
            PublicFrameworks.Add("OpenCL");
        } else if (Target.Platform == UnrealTargetPlatform.Linux) {
            if (Target.LinkType == TargetLinkType.Monolithic) {
                PublicAdditionalLibraries.Add(Path.Combine(Target.UEThirdPartySourceDirectory, "VHACD", "Lib", "Linux", Target.Architecture, "libVHACD.a"));
            } else {
                PublicAdditionalLibraries.Add(Path.Combine(Target.UEThirdPartySourceDirectory, "VHACD", "Lib", "Linux", Target.Architecture, "libVHACD_fPIC.a"));
            }
        } else {
            throw new Exception("Unexpected: Target.Platform == " + Target.Platform);
        }
    }
}
