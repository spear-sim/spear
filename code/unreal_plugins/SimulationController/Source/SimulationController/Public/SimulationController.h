// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <CoreMinimal.h>
#include <Modules/ModuleManager.h>

class UWorld;

class FSimulationControllerModule : public IModuleInterface
{
public:
    /** IModuleInterface implementation */
    /** This code will execute after your module is loaded into memory; the
     * exact timing is specified in the .uplugin file per-module
     */
    virtual void StartupModule() override;

    /** This function may be called during shutdown to clean up your module. For
     * modules that support dynamic reloading, we call this function before
     * unloading the module.
     */
    virtual void ShutdownModule() override;

    /**  Event Handlers  */
    void
    PostWorldInitializationEventHandler(UWorld* World,
                                        const UWorld::InitializationValues);

    void WorldCleanupEventHandler(UWorld* World,
                                  bool bSessionEnded,
                                  bool bCleanupResources);

    void ActorSpawnedEventHandler(AActor* InActor);

    void WorldInitializedActorsEventHandler(const FActorsInitializedParams& Params);
    void WorldBeginPlayEventHandler();

private :
    /** DelegateHandles corresponding to each Event Handler defined in this
     * class */
    FDelegateHandle PostWorldInitializationDelegateHandle;
    FDelegateHandle WorldCleanupDelegateHandle;
    FDelegateHandle ActorSpawnedDelegateHandle;

    FDelegateHandle WorldBeginPlayDelegateHandle;
    FDelegateHandle WorldInitializedActorsDelegateHandle;
};
