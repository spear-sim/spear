//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint32_t

#include <map>
#include <set>
#include <string>
#include <vector>

#include <Components/SkeletalMeshComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/EngineBaseTypes.h>    // ELevelTick
#include <Engine/EngineTypes.h>        // EBlendMode
#include <MaterialDomain.h>            // EMaterialDomain
#include <Materials/Material.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Materials/MaterialInterface.h>
#include <Math/Color.h>
#include <UObject/ObjectMacros.h>      // GENERATED_BODY, UCLASS, UPROPERTY
#include <UObject/UObjectGlobals.h>    // LoadObject

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpUnrealTypes/SpProxyComponentManager.h"

#include "SpPrimitiveProxyComponentManager.generated.h"

class UActorComponent;
class UMaterialInterface;
struct FActorComponentTickFunction;

USTRUCT(BlueprintType)
struct FComponentAndMaterialDesc
{
    GENERATED_BODY()

    UPROPERTY()
    uint32 Id = 0;
    UPROPERTY()
    uint64 Component = 0;
    UPROPERTY()
    uint64 Material = 0;

    // Optional debug info
    UPROPERTY()
    FString ComponentPtrString;
    UPROPERTY()
    FString ComponentPropertiesString;
    UPROPERTY()
    FString MaterialPtrString;
    UPROPERTY()
    FString MaterialPropertiesString;
};

// Don't instantiate ASpPrimitiveProxyComponentManager directly, use the derived classes below instead
UCLASS(ClassGroup="SPEAR", Config=Spear, HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpPrimitiveProxyComponentManager : public ASpProxyComponentManager
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    TArray<FComponentAndMaterialDesc> GetComponentAndMaterialDescs(bool bIncludeDebugInfo = false)
    {
        TArray<FComponentAndMaterialDesc> component_and_material_descs;

        for (auto& [id, desc] : id_to_component_and_material_desc_map_) {
            FComponentAndMaterialDesc component_and_material_desc;
            component_and_material_desc.Id = id;
            component_and_material_desc.Component = reinterpret_cast<uint64>(desc.component_);
            component_and_material_desc.Material = reinterpret_cast<uint64>(desc.material_);

            if (bIncludeDebugInfo) {
                component_and_material_desc.ComponentPtrString = Unreal::toFString(Std::toStringFromPtr(desc.component_));
                component_and_material_desc.ComponentPropertiesString = Unreal::toFString(Unreal::getObjectPropertiesAsString(desc.component_));
                component_and_material_desc.MaterialPtrString = Unreal::toFString(Std::toStringFromPtr(desc.material_));
                component_and_material_desc.MaterialPropertiesString = Unreal::toFString(Unreal::getObjectPropertiesAsString(desc.material_));
            }

            component_and_material_descs.Add(component_and_material_desc);
        }

        return component_and_material_descs;
    }

    // ASpProxyComponentManager interface
    void findAndDestroyAllProxyComponentTypes(const std::vector<AActor*>& actors) override
    {
        ASpProxyComponentManager::findAndDestroyProxyComponents<UStaticMeshComponent>(actors);
        ASpProxyComponentManager::findAndDestroyProxyComponents<USkeletalMeshComponent>(actors);
    }

    void findAndRegisterAllProxyComponentTypes(const std::vector<AActor*>& actors) override
    {
        ASpProxyComponentManager::findAndRegisterProxyComponents<UStaticMeshComponent>(actors, this);
        ASpProxyComponentManager::findAndRegisterProxyComponents<USkeletalMeshComponent>(actors, this);
    }

    void findAndUnregisterAllProxyComponentTypes(const std::vector<AActor*>& actors) override
    {
        ASpProxyComponentManager::findAndUnregisterProxyComponents<UStaticMeshComponent>(actors, this);
        ASpProxyComponentManager::findAndUnregisterProxyComponents<USkeletalMeshComponent>(actors, this);
    }

    void unregisterProxyComponents(const std::vector<std::string>& component_names) override
    {
        ASpProxyComponentManager::unregisterProxyComponents(component_names, this);
    }

    void initialize() override
    {
        ASpProxyComponentManager::initialize();
        InvisibleMaterial = LoadObject<UMaterialInterface>(nullptr, Unreal::toTCharPtr("/SpContent/Materials/M_Invisible.M_Invisible"));
    };

    void terminate() override
    {
        InvisibleMaterial = nullptr;
        ASpProxyComponentManager::terminate();
    }

    // CProxyComponentRegistryShouldRegister compile-time interface for deciding to register a USceneCompoennt
    bool shouldRegisterProxyComponent(USceneComponent* component)
    {
        SP_ASSERT(component);

        if (component->bHiddenInGame || !component->GetVisibleFlag()) {
            return false;
        }

        #if WITH_EDITOR
            if (!component->GetOwner() || component->GetOwner()->IsHiddenEd()) {
                return false;
            }
        #else
            if (!component->GetOwner() || component->GetOwner()->IsHidden()) {
                return false;
            }
        #endif

        return true;
    }

    // CProxyComponentRegistryShouldRegister compile-time interface deciding to unregister a USceneCompoennt
    bool shouldUnregisterProxyComponent(USceneComponent* component, void* user_data) const { return false; }

    // CProxyComponentRegistryCanRegister compile-time interface for registering UStaticMeshComponent
    void* registerProxyComponent(UStaticMeshComponent* proxy_component, UStaticMeshComponent* component)
    {
        ComponentRegistrationUserData* component_registration_user_data = createComponentRegistrationUserData();

        bool new_visibility;
        bool propagate_to_children;

        // set visibility to false while configuring our proxy component
        new_visibility = false;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        // configure component
        proxy_component->SetStaticMesh(component->GetStaticMesh());

        if (shouldProxyGeometryBeHiddenInViewport(proxy_component)) {
            proxy_component->SetVisibleInSceneCaptureOnly(true);
            proxy_component->bCastDynamicShadow = true;
            proxy_component->bCastStaticShadow = false;
            proxy_component->bAffectDistanceFieldLighting = false;
            proxy_component->bAffectDynamicIndirectLighting = false;
        }

        proxy_component->SetCanEverAffectNavigation(false);
        proxy_component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        proxy_component->SetGenerateOverlapEvents(false);

        for (int i = 0; i < component->GetNumMaterials(); i++) {
            if (shouldRegisterProxyComponentAndMaterial(component, component->GetMaterial(i))) {
                uint32_t component_and_material_desc_id = registerComponentAndMaterial(component, component->GetMaterial(i));
                SP_LOG("Registering component and material: ", Unreal::getStableName(component), " (material slot = ", i, ", ID = ", component_and_material_desc_id, ")");
                UMaterialInterface* material = createMaterial(component_and_material_desc_id, component, component->GetMaterial(i));
                SP_ASSERT(material);
                proxy_component->SetMaterial(i, material);
                component_registration_user_data->component_and_material_desc_ids_.insert(component_and_material_desc_id);
            } else {
                proxy_component->SetMaterial(i, InvisibleMaterial);
            }
        }

        // mark as dirty
        proxy_component->MarkRenderTransformDirty();
        proxy_component->MarkRenderStateDirty();

        // tick
        float delta_time = 0.0f;
        ELevelTick level_tick = ELevelTick::LEVELTICK_All;
        FActorComponentTickFunction* this_tick_function = nullptr;
        proxy_component->TickComponent(delta_time, level_tick, this_tick_function);

        // set visibility back to true now that we're finished configuring
        new_visibility = true;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        return component_registration_user_data;
    }

    // CProxyComponentRegistryCanRegister compile-time interface for registering USkeletalMeshComponent
    void* registerProxyComponent(USkeletalMeshComponent* proxy_component, USkeletalMeshComponent* component)
    {
        ComponentRegistrationUserData* component_registration_user_data = createComponentRegistrationUserData();

        bool new_visibility;
        bool propagate_to_children;

        // set visibility to false while configuring our proxy component
        new_visibility = false;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        // configure component
        proxy_component->SetSkeletalMesh(component->GetSkeletalMeshAsset());
        proxy_component->SetLeaderPoseComponent(component);

        if (shouldProxyGeometryBeHiddenInViewport(proxy_component)) {
            proxy_component->SetVisibleInSceneCaptureOnly(true);
            proxy_component->bCastDynamicShadow = true;
            proxy_component->bCastStaticShadow = false;
            proxy_component->bAffectDistanceFieldLighting = false;
            proxy_component->bAffectDynamicIndirectLighting = false;
        }

        proxy_component->SetCanEverAffectNavigation(false);
        proxy_component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        proxy_component->SetGenerateOverlapEvents(false);

        for (int i = 0; i < component->GetNumMaterials(); i++) {
            if (shouldRegisterProxyComponentAndMaterial(component, component->GetMaterial(i))) {
                uint32_t component_and_material_desc_id = registerComponentAndMaterial(component, component->GetMaterial(i));
                SP_LOG("Registering component and material: ", Unreal::getStableName(component), " (material slot = ", i, ", ID = ", component_and_material_desc_id, ")");
                UMaterialInterface* material = createMaterial(component_and_material_desc_id, component, component->GetMaterial(i));
                SP_ASSERT(material);
                proxy_component->SetMaterial(i, material);
                component_registration_user_data->component_and_material_desc_ids_.insert(component_and_material_desc_id);
            }
        }

        // mark as dirty
        proxy_component->RefreshBoneTransforms();
        proxy_component->InvalidateCachedBounds();
        proxy_component->MarkRenderTransformDirty();
        proxy_component->MarkRenderDynamicDataDirty();

        // tick
        float delta_time = 0.0f;
        ELevelTick level_tick = ELevelTick::LEVELTICK_All;
        FActorComponentTickFunction* this_tick_function = nullptr;
        proxy_component->TickComponent(delta_time, level_tick, this_tick_function);

        // set visibility back to true now that we're finished configuring
        new_visibility = true;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        return component_registration_user_data;
    }

    // CProxyComponentRegistryCanUnregister compile-time interface for unregistering all component types
    void unregisterProxyComponent(void* user_data)
    {
        ComponentRegistrationUserData* component_registration_user_data = static_cast<ComponentRegistrationUserData*>(user_data);
        SP_ASSERT(component_registration_user_data);

        for (auto component_and_material_desc_id : component_registration_user_data->component_and_material_desc_ids_) {
            ComponentAndMaterialDesc component_and_material_desc = id_to_component_and_material_desc_map_.at(component_and_material_desc_id);
            SP_LOG("Unregistering component and material: ", Unreal::getStableName(component_and_material_desc.component_), " (ID = ", component_and_material_desc_id, ")");
            unregisterComponentAndMaterial(component_and_material_desc_id);
        }

        destroyComponentRegistrationUserData(component_registration_user_data);
    }

    // scene capture components can use this to selectively add a manager to their allow lists
    virtual std::string getModalityName() const { SP_ASSERT(false); return ""; }; // don't instantiate ASpPrimitiveProxyComponentManager directly

protected:
    virtual bool shouldProxyGeometryBeHiddenInViewport(USceneComponent* component) const { return true; } // can be overridden by debug visualizers
    virtual bool shouldRegisterProxyComponentAndMaterial(USceneComponent* component, UMaterialInterface* material) const { SP_ASSERT(false); return false; } // don't instantiate ASpPrimitiveProxyComponentManager directly
    virtual UMaterialInterface* createMaterial(uint32_t id, USceneComponent* component, UMaterialInterface* material) { SP_ASSERT(false); return nullptr; } // don't instantiate ASpPrimitiveProxyComponentManager directly

private:
    struct ComponentRegistrationUserData
    {
        std::set<uint32_t> component_and_material_desc_ids_;
    };

    struct ComponentAndMaterialDesc
    {
        uint32_t id_ = 0;
        USceneComponent* component_ = nullptr;
        UMaterialInterface* material_ = nullptr;
    };

    UPROPERTY()
    UMaterialInterface* InvisibleMaterial = nullptr;

    uint32_t registerComponentAndMaterial(USceneComponent* component, UMaterialInterface* material)
    {
        uint32_t component_and_material_desc_id = getId(component_and_material_desc_id_initial_guess_, component_and_material_desc_ids_);
        component_and_material_desc_id_initial_guess_ = component_and_material_desc_id + 1;

        SP_ASSERT(!component_and_material_desc_ids_.contains(component_and_material_desc_id));
        SP_ASSERT(!Std::containsKey(id_to_component_and_material_desc_map_, component_and_material_desc_id));

        ComponentAndMaterialDesc component_and_material_desc;
        component_and_material_desc.id_ = component_and_material_desc_id;
        component_and_material_desc.component_ = component;
        component_and_material_desc.material_ = material;

        component_and_material_desc_ids_.insert(component_and_material_desc_id);
        Std::insert(id_to_component_and_material_desc_map_, component_and_material_desc_id, component_and_material_desc);

        return component_and_material_desc_id;
    }

    void unregisterComponentAndMaterial(uint32_t component_and_material_desc_id)
    {
        Std::remove(component_and_material_desc_ids_, component_and_material_desc_id);
        Std::remove(id_to_component_and_material_desc_map_, component_and_material_desc_id);
    }

    static ComponentRegistrationUserData* createComponentRegistrationUserData()
    {
        return new ComponentRegistrationUserData();
    }

    static void destroyComponentRegistrationUserData(ComponentRegistrationUserData* component_registration_user_data)
    {
        SP_ASSERT(component_registration_user_data);
        delete component_registration_user_data;
    }

    std::map<uint32_t, ComponentAndMaterialDesc> id_to_component_and_material_desc_map_;

    uint32_t component_and_material_desc_id_initial_guess_ = 1;
    std::set<uint32_t> component_and_material_desc_ids_;
};


UCLASS(ClassGroup="SPEAR", Config=Spear, HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpObjectIdsProxyComponentManager : public ASpPrimitiveProxyComponentManager
{
    GENERATED_BODY()

public:
    // AActor interface
    void BeginPlay() override
    {
        SP_LOG_CURRENT_FUNCTION();

        ASpPrimitiveProxyComponentManager::BeginPlay();
        EmissiveMaterial = nullptr;
    }

    std::string getModalityName() const override { return "object_ids"; }

protected:
    // ASpProxyComponentManager interface
    void initialize() override
    {
        ASpPrimitiveProxyComponentManager::initialize();
        EmissiveMaterial = LoadObject<UMaterialInterface>(nullptr, Unreal::toTCharPtr("/SpContent/Materials/M_Emissive.M_Emissive"));
    };

    void terminate() override
    {
        EmissiveMaterial = nullptr;
        ASpPrimitiveProxyComponentManager::terminate();
    }

    // ASpPrimitiveProxyComponentManager interface
    bool shouldRegisterProxyComponentAndMaterial(USceneComponent* component, UMaterialInterface* material) const override { return true; }

    // can't be const because UMaterialInstanceDynamic::Create(...) takes a non-const pointer
    UMaterialInterface* createMaterial(uint32_t id, USceneComponent* component, UMaterialInterface* material) override
    {
        SP_ASSERT(EmissiveMaterial);
        UMaterialInstanceDynamic* new_material = UMaterialInstanceDynamic::Create(EmissiveMaterial, this);
        SP_ASSERT(new_material);
        FColor color = getColor(id);
        new_material->SetVectorParameterValue("EmissiveColor", color);
        return new_material;
    }

private:
    UPROPERTY()
    UMaterialInterface* EmissiveMaterial = nullptr;

    static FColor getColor(uint32_t id)
    {
        SP_ASSERT(id > 0 && id <= 0xffffff);
        uint8_t r = static_cast<uint8_t>((id >> 16) & 0xff);
        uint8_t g = static_cast<uint8_t>((id >>  8) & 0xff);
        uint8_t b = static_cast<uint8_t>((id >>  0) & 0xff);
        uint8_t a = 0xff;
        return FColor(r, g, b, a);
    }
};

UCLASS(ClassGroup="SPEAR", Config=Spear, HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpObjectIdsVisualizerProxyComponentManager : public ASpObjectIdsProxyComponentManager
{
    GENERATED_BODY()

public:
    // AActor interface
    void BeginPlay() override
    {
        SP_LOG_CURRENT_FUNCTION();

        ASpPrimitiveProxyComponentManager::BeginPlay();
        DiffuseMaterial = nullptr;
    }

protected:
    // ASpProxyComponentManager interface
    void initialize() override
    {
        ASpPrimitiveProxyComponentManager::initialize();
        DiffuseMaterial = LoadObject<UMaterialInterface>(nullptr, Unreal::toTCharPtr("/SpContent/Materials/M_Diffuse.M_Diffuse"));
    };

    void terminate() override
    {
        DiffuseMaterial = nullptr;
        ASpPrimitiveProxyComponentManager::terminate();
    }

    // ASpPrimitiveProxyComponentManager interface
    bool shouldProxyGeometryBeHiddenInViewport(USceneComponent* component) const override { return false; }

    UMaterialInterface* createMaterial(uint32_t id, USceneComponent* component, UMaterialInterface* material) override
    {
        SP_ASSERT(DiffuseMaterial);
        UMaterialInstanceDynamic* new_material = UMaterialInstanceDynamic::Create(DiffuseMaterial, this);
        SP_ASSERT(new_material);
        FColor color = getColor(id);
        new_material->SetVectorParameterValue("BaseColor", color);
        return new_material;
    }

private:
    UPROPERTY()
    UMaterialInterface* DiffuseMaterial = nullptr;

    static FColor getColor(uint32_t id) { return FColor::MakeRandomColor(); }
};
