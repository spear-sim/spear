//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot.h"

#include <iostream>

#include <Interfaces/IPluginManager.h>

#include "CoreUtils/Assert.h"

void UrdfBot::StartupModule()
{
    std::cout << "[SPEAR | UrdfBot.cpp] UrdfBot::StartupModule" << std::endl;

    const FString PluginDir = IPluginManager::Get().FindPlugin("UrdfBot")->GetBaseDir();

	FString LibraryPath = FPaths::Combine(*PluginDir, TEXT("ThirdParty/mujoco/bin/"));
	DLLHandle = FPlatformProcess::GetDllHandle(*(LibraryPath + TEXT("mujoco.dll")));

    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void UrdfBot::ShutdownModule()
{
    if (DLLHandle) {
        FPlatformProcess::FreeDllHandle(DLLHandle);
        DLLHandle = nullptr;
    }
    std::cout << "[SPEAR | UrdfBot.cpp] UrdfBot::ShutdownModule" << std::endl;
}

IMPLEMENT_MODULE(UrdfBot, UrdfBot)
