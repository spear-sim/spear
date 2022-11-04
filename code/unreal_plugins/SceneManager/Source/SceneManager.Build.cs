// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class SceneManager : ModuleRules
{
	public SceneManager(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        bEnableExceptions = true;
		
        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "CoreUtils", "Engine", "PhysicsCore" });
        PrivateDependencyModuleNames.AddRange(new string[] {});
	}
}
