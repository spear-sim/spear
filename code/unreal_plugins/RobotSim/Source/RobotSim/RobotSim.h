// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

/**
 * @brief
 *
 */
class FRobotSimModule : public IModuleInterface {
public:
    /** IModuleInterface implementation */

    /**
     * @brief
     *
     */
    virtual void StartupModule() override;

    /**
     * @brief
     *
     */
    virtual void ShutdownModule() override;
};
