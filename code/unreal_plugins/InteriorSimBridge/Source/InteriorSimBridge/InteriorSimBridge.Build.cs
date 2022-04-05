// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class InteriorSimBridge : ModuleRules
{
	public InteriorSimBridge(ReadOnlyTargetRules Target) : base(Target)
	{
	    // This helps with build errors due to try/throw in asio caused by disabled exceptions (e.g. C4530 warning on windows)
        bEnableExceptions = true;

		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        
		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
                // ... add other public dependencies that you statically link with here ...
                "CoreUObject", "Engine", "InputCore", "ImageWrapper", 
                "ProceduralMeshComponent",
                "Landscape", "XmlParser", "APEX", "PhysX", "Foliage", "PhysicsCore",
                "NavigationSystem","PhysXVehicles",
                "PhysXVehicleLib"
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"InputCore", // This is required for using EKeys::W, etc 
				"CoreUObject",
				"ProceduralMeshComponent", "UMG", "PhysX", "NavigationSystem",
				"Engine",
				"RobotSim",
				"UnrealRL",
				"Renderer", 
				"RenderCore",
				"RHI"
				// ... add private dependencies that you statically link with here ...	
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
