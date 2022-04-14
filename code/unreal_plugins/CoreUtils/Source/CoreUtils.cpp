// Copyright Epic Games, Inc. All Rights Reserved.

#include "CoreUtils.h"

void CoreUtils::StartupModule()
{
    Config::initialize();
}

void CoreUtils::ShutdownModule()
{
    Config::terminate();
}

IMPLEMENT_MODULE(CoreUtils, CoreUtils)
