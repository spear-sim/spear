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

    const FString plugin_dir = IPluginManager::Get().FindPlugin("UrdfBot")->GetBaseDir();

    mujoco_dll_handle_ = FPlatformProcess::GetDllHandle(*FPaths::Combine(*plugin_dir, TEXT("ThirdParty/mujoco/bin/mujoco.dll")));

    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void UrdfBot::ShutdownModule()
{
    if (mujoco_dll_handle_) {
        FPlatformProcess::FreeDllHandle(mujoco_dll_handle_);
        mujoco_dll_handle_ = nullptr;
    }

    std::cout << "[SPEAR | UrdfBot.cpp] UrdfBot::ShutdownModule" << std::endl;
}

IMPLEMENT_MODULE(UrdfBot, UrdfBot)
