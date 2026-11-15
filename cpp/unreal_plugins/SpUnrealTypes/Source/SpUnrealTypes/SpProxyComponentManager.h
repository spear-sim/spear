//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <Containers/Array.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/EngineTypes.h>      // EEndPlayReason
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // int32, uint32
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpStableNameManager.h"
#include "SpCore/UnrealUtils.h"

#include "SpProxyComponentManager.generated.h"

class USpStableNameComponent;

//
// ASpProxyComponentManager is a UCLASS AActor that manages the lifecycle of a proxy component manager.
// It handles initialization, ticking, termination, and re-initialization across PIE transitions.
// Derived classes (e.g., ASpMeshProxyComponentManager) hold the actual CRTP proxy component manager
// and override the virtual hooks to dispatch into it.
//

UCLASS(Abstract, ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpProxyComponentManager : public AActor
{
    GENERATED_BODY()

public:
    ASpProxyComponentManager()
    {
        SP_LOG_CURRENT_FUNCTION();

        PrimaryActorTick.bCanEverTick = true;
        PrimaryActorTick.bTickEvenWhenPaused = true;
        PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;

        SpStableNameComponent = UnrealUtils::createComponentInsideOwnerConstructor<USpStableNameComponent>(this, "sp_stable_name_component");
        SP_ASSERT(SpStableNameComponent);
    }

    ~ASpProxyComponentManager() override
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    // AActor interface

    void BeginPlay() override
    {
        SP_LOG_CURRENT_FUNCTION();

        AActor::BeginPlay();

        // If bIsInitialized is true here, it means we're in the editor, the user called Initialize(), then the
        // user pressed play to initiate a PIE session. In this case, we need to defer initialization to Tick(...)
        // because some actors in the PIE session might not be initialized yet.
        if (bIsInitialized) {
            request_reinitialize_ = true;
        }

        // From now on, assume we're not initialized until we have a chance to initialize in Tick(...)
        bIsInitialized = false;
    }

    void EndPlay(const EEndPlayReason::Type end_play_reason) override
    {
        SP_LOG_CURRENT_FUNCTION();

        Terminate();
        AActor::EndPlay(end_play_reason);
    }

    void Tick(float delta_time) override
    {
        AActor::Tick(delta_time);

        //
        // We must take care to handle the case where an ASpProxyComponentManager actor in the editor world has
        // already been initialized, and we press play in the editor. In this case, bIsInitialized is duplicated
        // into the PIE world, but the internal state of the ASpProxyComponentManager is not. Proxy components are
        // marked as transient, so they are not duplicated into the PIE world either. So we re-initialize here and
        // create new proxy components in the PIE world. We can't do this re-initialization step in BeginPlay()
        // because there might be some actors and components that haven't been created yet.
        //

        if (request_reinitialize_) {
            request_reinitialize_ = false;
            initializeImpl();
            std::vector<AActor*> actors = UnrealUtils::findActors(GetWorld());
            unregisterAndDestroyProxyComponentsForAllTypes();
            requestCreateAndRegisterProxyComponentsForAllTypes(actors);
        } else if (IsInitialized() && bUpdateProxyComponentsEveryTick) {
            std::vector<AActor*> actors = UnrealUtils::findActors(GetWorld());
            requestUnregisterAndDestroyProxyComponentsForAllTypes(actors);
            requestCreateAndRegisterProxyComponentsForAllTypes(actors);
        }
    }

    bool ShouldTickIfViewportsOnly() const override { return true; }

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Initialize()
    {
        if (IsInitialized()) {
            return;
        }

        initializeImpl();
        std::vector<AActor*> actors = UnrealUtils::findActors(GetWorld());
        requestCreateAndRegisterProxyComponentsForAllTypes(actors);
    }

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Terminate()
    {
        if (!IsInitialized()) {
            return;
        }

        unregisterAndDestroyProxyComponentsForAllTypes();
        terminateImpl();
    }

    // Update() incrementally creates proxy components for new source components, and destroys proxy components
    // whose source components have been destroyed or are no longer visible or allowed. If bUpdateProxyComponentsEveryTick
    // is true, this happens automatically every tick. Otherwise, call Update() after spawning or destroying actors.

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Update()
    {
        if (!IsInitialized()) {
            return;
        }

        std::vector<AActor*> actors = UnrealUtils::findActors(GetWorld());
        requestUnregisterAndDestroyProxyComponentsForAllTypes(actors);
        requestCreateAndRegisterProxyComponentsForAllTypes(actors);
    }

    // Rebuild() destroys and recreates all proxy components. Update() doesn't refresh proxy components whose
    // source components have changed (e.g., a new mesh or new materials), so call Rebuild() after making such
    // changes. Note that proxy component IDs will be different after calling Rebuild().

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Rebuild()
    {
        if (!IsInitialized()) {
            return;
        }

        // Terminate()
        unregisterAndDestroyProxyComponentsForAllTypes();

        // Initialize()
        std::vector<AActor*> actors = UnrealUtils::findActors(GetWorld());
        requestCreateAndRegisterProxyComponentsForAllTypes(actors);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    bool IsInitialized() const { return is_initialized_; }

protected:
    // Called from Initialize(), Tick(...)
    virtual void initializeImpl()
    {
        is_initialized_ = true;
        bIsInitialized = true;
    }

    // Called from Terminate()
    virtual void terminateImpl()
    {
        is_initialized_ = false;
        bIsInitialized = false;
    }

    // Called from Initialize(), Tick(...), Update(), Rebuild()
    virtual void requestCreateAndRegisterProxyComponentsForAllTypes(const std::vector<AActor*>& actors) { SP_ASSERT(false); }

    // Called from Tick(...), Update()
    virtual void requestUnregisterAndDestroyProxyComponentsForAllTypes(const std::vector<AActor*>& actors) { SP_ASSERT(false); }

    // Called from Tick(...), Terminate(), Rebuild()
    virtual void unregisterAndDestroyProxyComponentsForAllTypes() { SP_ASSERT(false); }

private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpStableNameComponent* SpStableNameComponent = nullptr;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    bool bIsInitialized = false;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    bool bUpdateProxyComponentsEveryTick = true;

    bool request_reinitialize_ = false;
    bool is_initialized_ = false;
};
