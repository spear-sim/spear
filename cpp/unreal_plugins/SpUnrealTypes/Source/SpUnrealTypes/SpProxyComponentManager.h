//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
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
#include "SpCore/ProxyComponentManager.h"
#include "SpCore/SpStableNameManager.h"
#include "SpCore/UnrealUtils.h"

#include "SpProxyComponentManager.generated.h"

class UActorComponent;
class UClass;
class USpStableNameComponent;

//
// ASpProxyComponentManager is a UCLASS AActor that manages the lifecycle of a proxy component manager.
// It handles initialization, ticking, termination, and re-initialization across PIE transitions.
// Derived classes (e.g., ASpMeshProxyComponentManager) hold the actual CRTP proxy component manager,
// return it from getProxyComponentManager(), and override the virtual hooks to dispatch into it.
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

    //
    // AActor interface
    //

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
            Terminate(); // no-op unless Initialize() has already been called in the PIE world
            Initialize();
        } else if (IsInitialized() && bUpdateProxyComponentsEveryTick) {
            std::map<UClass*, std::vector<UActorComponent*>> non_proxy_components = getNonProxyComponentsForAllTypes();
            requestUnregisterAndDestroyProxyComponentsForAllTypes(non_proxy_components);
            requestCreateAndRegisterProxyComponentsForAllTypes(non_proxy_components);
        }
    }

    bool ShouldTickIfViewportsOnly() const override { return true; }

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Initialize()
    {
        if (IsInitialized()) {
            return;
        }

        // this logic is unique to Initialize()
        initializeImpl();
        initializeProxyComponentDescsForAllTypes();

        // this logic duplicates the logic in Rebuild() and the per-frame logic in Tick()
        std::map<UClass*, std::vector<UActorComponent*>> non_proxy_components = getNonProxyComponentsForAllTypes();
        requestCreateAndRegisterProxyComponentsForAllTypes(non_proxy_components);
    }

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Terminate()
    {
        if (!IsInitialized()) {
            return;
        }

        // this logic duplicates the logic in Rebuild()
        unregisterAndDestroyProxyComponentsForAllTypes(); // full update

        // this logic is unique to Terminate()
        terminateProxyComponentDescsForAllTypes();
        terminateImpl();
    }

    // Update() incrementally creates proxy components for new non-proxy components, and destroys proxy components
    // whose non-proxy components have been destroyed or are no longer visible or allowed. If bUpdateProxyComponentsEveryTick
    // is true, this happens automatically every tick. Otherwise, call Update() after spawning or destroying actors.

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Update()
    {
        if (!IsInitialized()) {
            return;
        }

        std::map<UClass*, std::vector<UActorComponent*>> non_proxy_components = getNonProxyComponentsForAllTypes();
        requestUnregisterAndDestroyProxyComponentsForAllTypes(non_proxy_components); // incremental update
        requestCreateAndRegisterProxyComponentsForAllTypes(non_proxy_components);
    }

    // Rebuild() destroys and recreates all proxy components. Update() doesn't refresh proxy components whose
    // non-proxy components have changed (e.g., a new mesh or new materials), so call Rebuild() after making such
    // changes. Note that proxy component IDs will be different after calling Rebuild(). Rebuild() unregisters all
    // proxy components immediately, ignoring the unregister delay, so after calling Rebuild(), flush enough frames
    // to ensure that rendered images no longer contain old IDs.

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Rebuild()
    {
        if (!IsInitialized()) {
            return;
        }

        std::map<UClass*, std::vector<UActorComponent*>> non_proxy_components = getNonProxyComponentsForAllTypes();
        unregisterAndDestroyProxyComponentsForAllTypes(); // full update
        requestCreateAndRegisterProxyComponentsForAllTypes(non_proxy_components);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    bool IsInitialized() const { return is_initialized_; }

protected:

    // Called from Initialize()
    virtual void initializeImpl()
    {
        is_initialized_ = true;
        bIsInitialized = true;
        getProxyComponentManager()->initialize();
    }

    // Called from Terminate()
    virtual void terminateImpl()
    {
        getProxyComponentManager()->terminate();
        is_initialized_ = false;
        bIsInitialized = false;
    }

    //
    // Called from initializeImpl(), terminateImpl(), and derived classes (e.g., ASpMeshProxyComponentManager). Derived
    // classes override these functions to return their proxy component manager, typically with a more derived return
    // type.
    //

    virtual ProxyComponentManagerBase* getProxyComponentManager() { SP_ASSERT(false); return nullptr; }
    virtual const ProxyComponentManagerBase* getProxyComponentManager() const { SP_ASSERT(false); return nullptr; }

    // Called from Initialize()
    virtual void initializeProxyComponentDescsForAllTypes() { SP_ASSERT(false); }

    // Called from Terminate()
    virtual void terminateProxyComponentDescsForAllTypes() { SP_ASSERT(false); }

    // Called from Initialize(), Tick(...), Update(), Rebuild()
    virtual std::map<UClass*, std::vector<UActorComponent*>> getNonProxyComponentsForAllTypes() const { SP_ASSERT(false); return std::map<UClass*, std::vector<UActorComponent*>>(); }

    // Called from Initialize(), Tick(...), Update(), Rebuild()
    virtual void requestCreateAndRegisterProxyComponentsForAllTypes(const std::map<UClass*, std::vector<UActorComponent*>>& non_proxy_components) { SP_ASSERT(false); }

    // Called from Tick(...), Update()
    virtual void requestUnregisterAndDestroyProxyComponentsForAllTypes(const std::map<UClass*, std::vector<UActorComponent*>>& non_proxy_components) { SP_ASSERT(false); }

    // Called from Terminate(), Rebuild()
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
