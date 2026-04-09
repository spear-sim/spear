//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint32_t

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
    int64 RawId = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    bool bIsValid = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString Component = "0x0";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ComponentStableName;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ComponentUnrealName;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString Material = "0x0";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString MaterialUnrealName;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString Actor = "0x0";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ActorName;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    bool bActorHasStableName = false;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ActorUnrealName;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ActorStableName;

    // Optional debug info
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ComponentPropertiesString = "{}";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString MaterialPropertiesString = "{}";
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString ActorPropertiesString = "{}";
};

//
// ASpMeshProxyComponentManager holds a MeshProxyComponentManager* and dispatches the
// ASpProxyComponentManager virtual hooks into it. Derived classes set the pointer via
// setMeshProxyComponentManager() and implement IMeshProxyComponentManager.
//

UCLASS(Abstract, ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpMeshProxyComponentManager : public ASpProxyComponentManager
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    TArray<FMeshProxyGeometryDesc> GetMeshProxyGeometryDescs(bool bIncludeDebugInfo = false)
    {
        SP_ASSERT(mesh_proxy_component_manager_);

        TArray<FMeshProxyGeometryDesc> mesh_proxy_geometry_descs;

        // add 0 desc explicitly
        mesh_proxy_geometry_descs.Add(FMeshProxyGeometryDesc());

        for (auto& [id, desc] : mesh_proxy_component_manager_->getMeshProxyGeometryDescs()) {
            FMeshProxyGeometryDesc mesh_proxy_geometry_desc;
            mesh_proxy_geometry_desc.RawId = id;
            mesh_proxy_geometry_desc.bIsValid = desc.is_valid_;

            if (desc.is_valid_) {
                mesh_proxy_geometry_desc.Component = Unreal::toFString(Std::toStringFromPtr(desc.component_));
                mesh_proxy_geometry_desc.ComponentStableName = Unreal::toFString(UnrealUtils::getStableName(desc.component_));
                mesh_proxy_geometry_desc.ComponentUnrealName = desc.component_->GetName();

                mesh_proxy_geometry_desc.Material = Unreal::toFString(Std::toStringFromPtr(desc.material_));
                mesh_proxy_geometry_desc.MaterialUnrealName = desc.material_->GetName();

                AActor* actor = desc.component_->GetOwner();
                if (actor) {
                    mesh_proxy_geometry_desc.Actor = Unreal::toFString(Std::toStringFromPtr(actor));
                    mesh_proxy_geometry_desc.ActorName = Unreal::toFString(UnrealUtils::tryGetStableName(actor));
                    mesh_proxy_geometry_desc.ActorUnrealName = actor->GetName();
                    bool has_stable_name = UnrealUtils::hasStableName(actor);
                    mesh_proxy_geometry_desc.bActorHasStableName = has_stable_name;
                    if (has_stable_name) {
                        mesh_proxy_geometry_desc.ActorStableName = Unreal::toFString(UnrealUtils::getStableName(actor));
                    }
                }

                if (bIncludeDebugInfo) {
                    mesh_proxy_geometry_desc.ComponentPropertiesString = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(desc.component_));
                    mesh_proxy_geometry_desc.MaterialPropertiesString = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(desc.material_));
                    mesh_proxy_geometry_desc.ActorPropertiesString = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(desc.component_->GetOwner()));
                }
            }

            mesh_proxy_geometry_descs.Add(mesh_proxy_geometry_desc);
        }

        return mesh_proxy_geometry_descs;
    }

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    int32 UnregisterDelayFrames = 3;

protected:
    void setMeshProxyComponentManager(MeshProxyComponentManager* mesh_proxy_component_manager)
    {
        mesh_proxy_component_manager_ = mesh_proxy_component_manager;
    }

    // ASpProxyComponentManager interface

    void initializeImpl() override
    {
        SP_ASSERT(mesh_proxy_component_manager_);
        ASpProxyComponentManager::initializeImpl();
        mesh_proxy_component_manager_->setUnregisterDelayFrames(UnregisterDelayFrames);
    }

    void terminateImpl() override
    {
        ASpProxyComponentManager::terminateImpl();
    }

    void requestCreateAndRegisterProxyComponentsForAllTypes(const std::vector<AActor*>& actors) override
    {
        SP_ASSERT(mesh_proxy_component_manager_);
        mesh_proxy_component_manager_->requestCreateAndRegisterProxyComponents<UStaticMeshComponent>(actors);
        mesh_proxy_component_manager_->requestCreateAndRegisterProxyComponents<USkeletalMeshComponent>(actors);
    }

    void requestUnregisterAndDestroyProxyComponentsForAllTypes(const std::vector<AActor*>& actors) override
    {
        SP_ASSERT(mesh_proxy_component_manager_);
        mesh_proxy_component_manager_->requestUnregisterAndDestroyProxyComponents<UStaticMeshComponent>(actors);
        mesh_proxy_component_manager_->requestUnregisterAndDestroyProxyComponents<USkeletalMeshComponent>(actors);
    }

    void requestDestroyProxyComponentsForAllTypes(const std::vector<AActor*>& actors) override
    {
        SP_ASSERT(mesh_proxy_component_manager_);
        mesh_proxy_component_manager_->requestDestroyProxyComponents<UStaticMeshComponent>(actors);
        mesh_proxy_component_manager_->requestDestroyProxyComponents<USkeletalMeshComponent>(actors);
    }

    void unregisterAllProxyComponents() override
    {
        SP_ASSERT(mesh_proxy_component_manager_);
        mesh_proxy_component_manager_->unregisterAllProxyComponents();
    }

private:
    MeshProxyComponentManager* mesh_proxy_component_manager_ = nullptr;
};


UCLASS(ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpObjectIdsProxyComponentManager : public ASpMeshProxyComponentManager, public IMeshProxyComponentManager
{
    GENERATED_BODY()

public:
    ASpObjectIdsProxyComponentManager()
    {
        setMeshProxyComponentManager(&mesh_proxy_component_manager_);
    }

    void BeginPlay() override
    {
        SP_LOG_CURRENT_FUNCTION();
        ASpMeshProxyComponentManager::BeginPlay();
        EmissiveMaterial = nullptr;
    }

protected:
    void initializeImpl() override
    {
        ASpMeshProxyComponentManager::initializeImpl();
        EmissiveMaterial = LoadObject<UMaterialInterface>(nullptr, Unreal::toTCharPtr("/SpContent/Materials/M_Emissive.M_Emissive"));
    }

    void terminateImpl() override
    {
        EmissiveMaterial = nullptr;
        ASpMeshProxyComponentManager::terminateImpl();
    }

    // IMeshProxyComponentManager interface

    UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* component, UMaterialInterface* material) override
    {
        SP_ASSERT(EmissiveMaterial);
        UMaterialInstanceDynamic* new_material = UMaterialInstanceDynamic::Create(EmissiveMaterial, this);
        SP_ASSERT(new_material);
        FLinearColor color = getLinearColorImpl(id);
        new_material->SetVectorParameterValue("EmissiveColor", color);
        return new_material;
    }

    bool shouldProxyComponentBeHiddenInViewport(USceneComponent* component) const override { return true; }

private:
    MeshProxyComponentManager mesh_proxy_component_manager_ = MeshProxyComponentManager(this, this);

    UPROPERTY()
    UMaterialInterface* EmissiveMaterial = nullptr;

    static FLinearColor getLinearColorImpl(uint32_t id)
    {
        SP_ASSERT(id > 0 && id <= 0xffffff);
        float r = static_cast<float>((id >> 16) & 0xff) / 255.0f;
        float g = static_cast<float>((id >>  8) & 0xff) / 255.0f;
        float b = static_cast<float>((id >>  0) & 0xff) / 255.0f;
        float a = 1.0f;
        return FLinearColor(r, g, b, a);
    }
};


UCLASS(ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpObjectIdsVisualizerProxyComponentManager : public ASpObjectIdsProxyComponentManager
{
    GENERATED_BODY()

public:
    void BeginPlay() override
    {
        SP_LOG_CURRENT_FUNCTION();
        ASpMeshProxyComponentManager::BeginPlay();
        DiffuseMaterial = nullptr;
    }

protected:
    void initializeImpl() override
    {
        ASpMeshProxyComponentManager::initializeImpl();
        DiffuseMaterial = LoadObject<UMaterialInterface>(nullptr, Unreal::toTCharPtr("/SpContent/Materials/M_Diffuse.M_Diffuse"));
    }

    void terminateImpl() override
    {
        DiffuseMaterial = nullptr;
        ASpMeshProxyComponentManager::terminateImpl();
    }

    // IMeshProxyComponentManager interface

    UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* component, UMaterialInterface* material) override
    {
        SP_ASSERT(DiffuseMaterial);
        UMaterialInstanceDynamic* new_material = UMaterialInstanceDynamic::Create(DiffuseMaterial, this);
        SP_ASSERT(new_material);
        FColor color = getColorImpl(id);
        new_material->SetVectorParameterValue("BaseColor", color);
        return new_material;
    }

    bool shouldProxyComponentBeHiddenInViewport(USceneComponent* component) const override { return false; }

private:
    UPROPERTY()
    UMaterialInterface* DiffuseMaterial = nullptr;

    static FColor getColorImpl(uint32_t id) { return FColor::MakeRandomColor(); }
};
