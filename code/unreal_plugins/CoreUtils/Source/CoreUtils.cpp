// Copyright Epic Games, Inc. All Rights Reserved.

#include "CoreUtils.h"

void CoreUtils::StartupModule() {
    // Initialize config system
    Config::initialize();
}

void CoreUtils::ShutdownModule() {
    // Terminate config system as we will not use it anymore
    Config::terminate();
}

IMPLEMENT_MODULE(CoreUtils, CoreUtils)
