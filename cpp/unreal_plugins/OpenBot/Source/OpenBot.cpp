//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "OpenBot.h"

#include "Assert/Assert.h"

void OpenBot::StartupModule()
{
    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void OpenBot::ShutdownModule() {}

IMPLEMENT_MODULE(OpenBot, OpenBot)
