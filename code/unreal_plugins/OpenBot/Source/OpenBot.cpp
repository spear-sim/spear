// Copyright Epic Games, Inc. All Rights Reserved.

#include "OpenBot.h"

#include "Assert/Assert.h"

void OpenBot::StartupModule()
{
    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void OpenBot::ShutdownModule()
{}

IMPLEMENT_MODULE(OpenBot, OpenBot)
