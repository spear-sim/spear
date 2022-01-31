// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

DECLARE_LOG_CATEGORY_EXTERN(LogInteriorSimBridge, Log, All);

class UWorld;

class FInteriorSimBridgeModule : public IModuleInterface
{
public:
    /** IModuleInterface implementation */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

    /**  Event Handlers  */
    void
    PostWorldInitializationEventHandler(UWorld* World,
                                        const UWorld::InitializationValues);

    void WorldCleanupEventHandler(UWorld* World,
                                  bool bSessionEnded,
                                  bool bCleanupResources);

    void ActorSpawnedEventHandler(AActor* InActor);

    void WorldInitializedActorsEventHandler(
        const UWorld::FActorsInitializedParams& ActorsInitializedParams);

private:
    UWorld* WorldInstance = nullptr;
};
