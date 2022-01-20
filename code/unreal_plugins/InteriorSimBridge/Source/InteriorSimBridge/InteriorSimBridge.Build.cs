// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class InteriorSimBridge : ModuleRules
{
	public InteriorSimBridge(ReadOnlyTargetRules Target) : base(Target)
	{
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
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"InputCore", // This is required for using EKeys::W, etc 
				"CoreUObject",
				"Engine",
				"RobotSim",
				"UnrealRL",
				"RHI",
				"Renderer",
				"RenderCore"
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
