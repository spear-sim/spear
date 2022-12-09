//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#pragma once

#include <CoreMinimal.h>
#include <Modules/ModuleManager.h>

class CoreUtils : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;
};
