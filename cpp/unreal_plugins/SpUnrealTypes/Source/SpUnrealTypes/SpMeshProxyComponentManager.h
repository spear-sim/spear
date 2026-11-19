//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint32_t

#include <map>
#include <vector>

#include <Components/SkeletalMeshComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Containers/Array.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/EngineTypes.h>      // EBlendMode
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // int32, int64
#include <MaterialDomain.h>          // EMaterialDomain
#include <Materials/Material.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Materials/MaterialInterface.h>
#include <Math/Color.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY
#include <UObject/UObjectGlobals.h>  // LoadObject

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/MeshProxyComponentManager.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

#include "SpUnrealTypes/SpProxyComponentManager.h"

#include "SpMeshProxyComponentManager.generated.h"

class UActorComponent;
class UClass;
class USceneComponent;

//
// FMeshProxyGeometryDesc is the UE-friendly representation of a MeshProxyGeometryDesc, returned by
// ASpMeshProxyComponentManager::GetMeshProxyGeometryDescs().
//

USTRUCT(BlueprintType)
struct FMeshProxyGeometryDesc
{
    GENERATED_BODY()
public:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    int64 ProxyId = 0; // Id gets converted to ID by Unreal's JSON serialization code because of an FName collision

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    bool bIsValid = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString NonProxyComponent = "0x0";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString NonProxyComponentStableName;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString NonProxyComponentUnrealName;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString NonProxyMaterial = "0x0";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString NonProxyMaterialUnrealName;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString Actor = "0x0";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ActorName;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ActorUnrealName;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ActorStableName;

    // Optional debug info
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString NonProxyComponentPropertiesString = "{}";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString NonProxyMaterialPropertiesString = "{}";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ActorPropertiesString = "{}";
};

//
// ASpMeshProxyComponentManager is an abstract ASpProxyComponentManager for UStaticMeshComponent and
// USkeletalMeshComponent. It implements the ASpProxyComponentManager virtual hooks by calling into a
// MeshProxyComponentManager for each of these component types, and exposes the MeshProxyComponentManager's state
// (proxy geometry descs, allow and ignore lists, unregister delay) to Python. Derived classes own a concrete
// MeshProxyComponentManager subclass and return it from getProxyComponentManager().
//

UCLASS(Abstract, ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpMeshProxyComponentManager : public ASpProxyComponentManager
{
    GENERATED_BODY()

public:

    // AActor interface
    void Tick(float delta_time) override
    {
        ASpProxyComponentManager::Tick(delta_time);

        // only need to update these variables for debugging in the editor
        #if WITH_EDITOR // defined in an auto-generated header
            if (IsInitialized() && bUpdateDebugInfoEveryTick) {
                UnregisterDelayFrames = GetUnregisterDelayFrames();
                MeshProxyGeometryDescs = GetMeshProxyGeometryDescs();
            }
        #endif
    }

    //
    // Public interface for Python users
    //

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    TArray<FMeshProxyGeometryDesc> GetMeshProxyGeometryDescs(bool bIncludeDebugInfo = false) const
    {
        SP_ASSERT(IsInitialized());

        TArray<FMeshProxyGeometryDesc> mesh_proxy_geometry_descs;

        // explicitly add empty desc at index 0
        mesh_proxy_geometry_descs.Add(FMeshProxyGeometryDesc());

        for (auto& [id, desc] : getProxyComponentManager()->getMeshProxyGeometryDescs()) {
            FMeshProxyGeometryDesc mesh_proxy_geometry_desc;
            mesh_proxy_geometry_desc.ProxyId = id;
            mesh_proxy_geometry_desc.bIsValid = desc.is_valid_;

            // It is possible for desc.is_valid_ to be true, but for Unreal::toPtr(desc.non_proxy_component_) or
            // Unreal::toPtr(desc.non_proxy_material_) to return nullptr, e.g., if an actor gets destroyed after
            // SpProxyComponentManager has ticked, but before this function is called.

            if (desc.is_valid_) {
                USceneComponent* component = Unreal::toPtr(desc.non_proxy_component_);
                if (component) {
                    mesh_proxy_geometry_desc.NonProxyComponent = Unreal::toFString(Std::toStringFromPtr(component));
                    mesh_proxy_geometry_desc.NonProxyComponentStableName = Unreal::toFString(UnrealUtils::getStableName(component));
                    mesh_proxy_geometry_desc.NonProxyComponentUnrealName = component->GetName();
                    if (bIncludeDebugInfo) {
                        mesh_proxy_geometry_desc.NonProxyComponentPropertiesString = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(component));
                    }

                    AActor* actor = component->GetOwner();
                    SP_ASSERT(Unreal::isValid(actor));
                    mesh_proxy_geometry_desc.Actor = Unreal::toFString(Std::toStringFromPtr(actor));
                    mesh_proxy_geometry_desc.ActorName = Unreal::toFString(UnrealUtils::getStableName(actor, true)); // include_unreal_name=true
                    mesh_proxy_geometry_desc.ActorUnrealName = actor->GetName();
                    mesh_proxy_geometry_desc.ActorStableName = Unreal::toFString(UnrealUtils::getStableName(actor, false)); // include_unreal_name=false
                    if (bIncludeDebugInfo) {
                        mesh_proxy_geometry_desc.ActorPropertiesString = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(actor));
                    }
                }

                UMaterialInterface* material = Unreal::toPtr(desc.non_proxy_material_);
                if (material) {
                    mesh_proxy_geometry_desc.NonProxyMaterial = Unreal::toFString(Std::toStringFromPtr(material));
                    mesh_proxy_geometry_desc.NonProxyMaterialUnrealName = material->GetName();
                    if (bIncludeDebugInfo) {
                        mesh_proxy_geometry_desc.NonProxyMaterialPropertiesString = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(material));
                    }
                }
            }

            mesh_proxy_geometry_descs.Add(mesh_proxy_geometry_desc);
        }

        return mesh_proxy_geometry_descs;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    TArray<AActor*> GetAllowedActors() const
    {
        SP_ASSERT(IsInitialized());
        return Unreal::toTArray(getProxyComponentManager()->getAllowedActors());
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void SetAllowedActors(const TArray<AActor*>& AllowedActors)
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->setAllowedActors(Unreal::toStdVector(AllowedActors));
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    TArray<USceneComponent*> GetAllowedComponents() const
    {
        SP_ASSERT(IsInitialized());
        return Unreal::toTArray(getProxyComponentManager()->getAllowedComponents());
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void SetAllowedComponents(const TArray<USceneComponent*>& AllowedComponents)
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->setAllowedComponents(Unreal::toStdVector(AllowedComponents));
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    TArray<AActor*> GetIgnoredActors() const
    {
        SP_ASSERT(IsInitialized());
        return Unreal::toTArray(getProxyComponentManager()->getIgnoredActors());
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void SetIgnoredActors(const TArray<AActor*>& IgnoredActors)
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->setIgnoredActors(Unreal::toStdVector(IgnoredActors));
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    TArray<USceneComponent*> GetIgnoredComponents() const
    {
        SP_ASSERT(IsInitialized());
        return Unreal::toTArray(getProxyComponentManager()->getIgnoredComponents());
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void SetIgnoredComponents(const TArray<USceneComponent*>& IgnoredComponents)
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->setIgnoredComponents(Unreal::toStdVector(IgnoredComponents));
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    int32 GetUnregisterDelayFrames() const
    {
        SP_ASSERT(IsInitialized());
        return getProxyComponentManager()->getUnregisterDelayFrames();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void SetUnregisterDelayFrames(int32 InUnregisterDelayFrames)
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->setUnregisterDelayFrames(InUnregisterDelayFrames);
    }

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bUpdateDebugInfoEveryTick = false;

protected:

    //
    // ASpProxyComponentManager interface
    //

    void initializeImpl() override
    {
        ASpProxyComponentManager::initializeImpl();
    }

    void terminateImpl() override
    {
        #if WITH_EDITOR // defined in an auto-generated header
            UnregisterDelayFrames = -1;
            MeshProxyGeometryDescs.Empty();
        #endif

        ASpProxyComponentManager::terminateImpl();
    }

    MeshProxyComponentManager* getProxyComponentManager() override { SP_ASSERT(false); return nullptr; }
    const MeshProxyComponentManager* getProxyComponentManager() const override { SP_ASSERT(false); return nullptr; }

    void initializeProxyComponentDescsForAllTypes() override
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->initializeProxyComponentDescs<UStaticMeshComponent>();
        getProxyComponentManager()->initializeProxyComponentDescs<USkeletalMeshComponent>();
    }

    void terminateProxyComponentDescsForAllTypes() override
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->terminateProxyComponentDescs<UStaticMeshComponent>();
        getProxyComponentManager()->terminateProxyComponentDescs<USkeletalMeshComponent>();
    }

    std::map<UClass*, std::vector<UActorComponent*>> getNonProxyComponentsForAllTypes() const override
    {
        std::vector<AActor*> actors = UnrealUtils::findActors(GetWorld());
        std::map<UClass*, std::vector<UActorComponent*>> non_proxy_components = {{UStaticMeshComponent::StaticClass(), {}}, {USkeletalMeshComponent::StaticClass(), {}}};

        for (auto actor : actors) {
            for (auto component : UnrealUtils::getComponents(actor)) {
                if (!Std::contains(Unreal::toStdString(component->GetName()), "__SP_PROXY_COMPONENT__")) {
                    for (auto& [component_class, non_proxy_components_for_class] : non_proxy_components) {
                        if (component->IsA(component_class)) {
                            non_proxy_components_for_class.push_back(component);
                        }
                    }
                }
            }
        }

        return non_proxy_components;
    }

    void requestCreateAndRegisterProxyComponentsForAllTypes(const std::map<UClass*, std::vector<UActorComponent*>>& non_proxy_components) override
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->requestCreateAndRegisterProxyComponents<UStaticMeshComponent>(non_proxy_components.at(UStaticMeshComponent::StaticClass()));
        getProxyComponentManager()->requestCreateAndRegisterProxyComponents<USkeletalMeshComponent>(non_proxy_components.at(USkeletalMeshComponent::StaticClass()));
    }

    void requestUnregisterAndDestroyProxyComponentsForAllTypes(const std::map<UClass*, std::vector<UActorComponent*>>& non_proxy_components) override
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->requestDecrementAndUnregisterProxyComponents();
        getProxyComponentManager()->requestUnregisterAndDestroyProxyComponents<UStaticMeshComponent>(non_proxy_components.at(UStaticMeshComponent::StaticClass()));
        getProxyComponentManager()->requestUnregisterAndDestroyProxyComponents<USkeletalMeshComponent>(non_proxy_components.at(USkeletalMeshComponent::StaticClass()));
    }

    void unregisterAndDestroyProxyComponentsForAllTypes() override
    {
        SP_ASSERT(IsInitialized());
        getProxyComponentManager()->unregisterAndDestroyProxyComponents();
    }

private:
    UPROPERTY(Transient, VisibleAnywhere, Category="SPEAR")
    int32 UnregisterDelayFrames = -1;

    UPROPERTY(Transient, VisibleAnywhere, Category="SPEAR")
    TArray<FMeshProxyGeometryDesc> MeshProxyGeometryDescs;

};

//
// ASpObjectIdsProxyComponentManager and ASpObjectIdsVisualizerProxyComponentManager choose a proxy strategy by owning an
// ObjectIdsMeshProxyComponentManager or an ObjectIdsVisualizerMeshProxyComponentManager, see MeshProxyComponentManager.h.
// ASpObjectIdsProxyComponentManager renders proxy components only in scene captures, and is used by SegmentationService
// (python/spear/services/segmentation_service.py). ASpObjectIdsVisualizerProxyComponentManager renders proxy components
// with random colors in the viewport, so users can inspect the segmentation in the editor.
//

UCLASS(ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpObjectIdsProxyComponentManager : public ASpMeshProxyComponentManager
{
    GENERATED_BODY()

protected:
    MeshProxyComponentManager* getProxyComponentManager() override { return &object_ids_mesh_proxy_component_manager_; }
    const MeshProxyComponentManager* getProxyComponentManager() const override { return &object_ids_mesh_proxy_component_manager_; }

private:
    ObjectIdsMeshProxyComponentManager object_ids_mesh_proxy_component_manager_ = ObjectIdsMeshProxyComponentManager(this);
};


UCLASS(ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpObjectIdsVisualizerProxyComponentManager : public ASpMeshProxyComponentManager
{
    GENERATED_BODY()

protected:
    MeshProxyComponentManager* getProxyComponentManager() override { return &object_ids_visualizer_mesh_proxy_component_manager_; }
    const MeshProxyComponentManager* getProxyComponentManager() const override { return &object_ids_visualizer_mesh_proxy_component_manager_; }

private:
    ObjectIdsVisualizerMeshProxyComponentManager object_ids_visualizer_mesh_proxy_component_manager_ = ObjectIdsVisualizerMeshProxyComponentManager(this);
};
