//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // uint32_t

#include <algorithm> // std::min
#include <map>
#include <set>
#include <string>
#include <vector>

#include <Components/SkeletalMeshComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/EngineBaseTypes.h>    // ELevelTick
#include <Materials/MaterialInstanceDynamic.h>
#include <Materials/MaterialInterface.h>
#include <Math/Color.h>
#include <UObject/ObjectMacros.h>      // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpUnrealTypes/SpProxyComponentManager.h"

#include "SpComponentAndMaterialIdProxyComponentManager.generated.h"

class UActorComponent;
class UMaterialInterface;
struct FActorComponentTickFunction;

UCLASS(ClassGroup="SPEAR", Config=Spear, HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpComponentAndMaterialIdProxyComponentManager : public ASpProxyComponentManager
{
    GENERATED_BODY()
private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    UMaterialInterface* Material;

public:
    // AActor interface
    void BeginPlay() override
    {
        SP_LOG_CURRENT_FUNCTION();

        ASpProxyComponentManager::BeginPlay();

        Material = nullptr;
    }

protected:
    // ASpProxyComponentManager interface
    void initialize() override
    {
        ASpProxyComponentManager::initialize();
        Material = LoadObject<UMaterialInterface>(nullptr, *Unreal::toFString("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    };

    void terminate() override
    {
        Material = nullptr;
        ASpProxyComponentManager::terminate();
    }

    void findAndDestroyAllProxyComponentTypes(const std::vector<AActor*>& actors) override
    {
        ASpProxyComponentManager::findAndDestroyAllProxyComponentTypes(actors);
        ASpProxyComponentManager::findAndDestroyProxyComponents<UStaticMeshComponent>(actors);
        ASpProxyComponentManager::findAndDestroyProxyComponents<USkeletalMeshComponent>(actors);
    }

    void findAndRegisterAllProxyComponentTypes(const std::vector<AActor*>& actors) override
    {
        ASpProxyComponentManager::findAndRegisterAllProxyComponentTypes(actors);
        ASpProxyComponentManager::findAndRegisterProxyComponents<UStaticMeshComponent>(actors, this);
        ASpProxyComponentManager::findAndRegisterProxyComponents<USkeletalMeshComponent>(actors, this);
    }

    void findAndUnregisterAllProxyComponentTypes(const std::vector<AActor*>& actors) override
    {
        ASpProxyComponentManager::findAndUnregisterProxyComponents<UStaticMeshComponent>(actors, this);
        ASpProxyComponentManager::findAndUnregisterProxyComponents<USkeletalMeshComponent>(actors, this);
        ASpProxyComponentManager::findAndUnregisterAllProxyComponentTypes(actors);
    }

    void unregisterProxyComponents(const std::vector<std::string>& component_names) override
    {
        ASpProxyComponentManager::unregisterProxyComponents(component_names, this);
        ASpProxyComponentManager::unregisterProxyComponents(component_names);
    }

public:
    // CProxyComponentRegistryShouldRegister compile-time interface for filtering UStaticMeshComponent (optional)
    bool shouldRegisterProxyComponent(UStaticMeshComponent* component)
    {
        return true;
    }

    // CProxyComponentRegistryCanRegister compile-time interface for registering UStaticMeshComponent
    void* registerProxyComponent(UStaticMeshComponent* proxy_component, UStaticMeshComponent* component)
    {
        std::set<uint32_t>* component_and_material_desc_ids = createSet();

        bool new_visibility;
        bool propagate_to_children;

        // set visibility to false
        new_visibility = false;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        // configure component
        proxy_component->SetStaticMesh(component->GetStaticMesh());

        for (int i = 0; i < component->GetNumMaterials(); i++) {
            uint32_t component_and_material_desc_id = registerComponentAndMaterial(component, component->GetMaterial(i));
            UMaterialInstanceDynamic* material_instance_dynamic = createMaterialInstanceDynamic(component_and_material_desc_id);
            SP_ASSERT(material_instance_dynamic);
            proxy_component->SetMaterial(i, material_instance_dynamic);
            component_and_material_desc_ids->insert(component_and_material_desc_id);
        }

        // mark as dirty
        proxy_component->MarkRenderTransformDirty();
        proxy_component->MarkRenderStateDirty();

        // tick
        float delta_time = 0.0f;
        ELevelTick level_tick = ELevelTick::LEVELTICK_All;
        FActorComponentTickFunction* this_tick_function = nullptr;
        proxy_component->TickComponent(delta_time, level_tick, this_tick_function);

        // set visibility to true
        new_visibility = true;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        return component_and_material_desc_ids;
    }

    // CProxyComponentRegistryCanRegister compile-time interface for registering USkeletalMeshComponent
    void* registerProxyComponent(USkeletalMeshComponent* proxy_component, USkeletalMeshComponent* component)
    {
        std::set<uint32_t>* component_and_material_desc_ids = createSet();

        bool new_visibility;
        bool propagate_to_children;

        // set visibility to false
        new_visibility = false;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        // configure component
        proxy_component->SetSkeletalMesh(component->GetSkeletalMeshAsset());
        proxy_component->SetLeaderPoseComponent(component);

        for (int i = 0; i < component->GetNumMaterials(); i++) {
            uint32_t component_and_material_desc_id = registerComponentAndMaterial(component, component->GetMaterial(i));
            UMaterialInstanceDynamic* material_instance_dynamic = createMaterialInstanceDynamic(component_and_material_desc_id);
            SP_ASSERT(material_instance_dynamic);
            proxy_component->SetMaterial(i, material_instance_dynamic);
            component_and_material_desc_ids->insert(component_and_material_desc_id);
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

        // set visibility to true
        new_visibility = true;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        return component_and_material_desc_ids;
    }

    // CProxyComponentRegistryCanUnregister compile-time interface for unregistering all component types
    void unregisterProxyComponent(void* user_data)
    {
        std::set<uint32_t>* component_and_material_desc_ids = static_cast<std::set<uint32_t>*>(user_data);

        SP_ASSERT(component_and_material_desc_ids);
        for (auto component_and_material_desc_id : *component_and_material_desc_ids) {
            unregisterComponentAndMaterial(component_and_material_desc_id);
        }

        destroySet(component_and_material_desc_ids);
    }


private:
    struct ComponentAndMaterialDesc
    {
        uint32_t id_ = 0;
        USceneComponent* component_ = nullptr;
        UMaterialInterface* material_ = nullptr;
    };

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

    UMaterialInstanceDynamic* createMaterialInstanceDynamic(uint32_t id)
    {
        SP_ASSERT(Material);
        UMaterialInstanceDynamic* material_instance_dynamic = UMaterialInstanceDynamic::Create(Material, this);
        SP_ASSERT(material_instance_dynamic);
        FColor color = FColor::MakeRandomColor(); // TODO: set color according to id
        material_instance_dynamic->SetVectorParameterValue("Color", color);
        return material_instance_dynamic;
    }

    template <typename TVoid = void>
    std::set<uint32_t>* createSet()
    {
        return new std::set<uint32_t>(); 
    }

    template <typename TVoid = void>
    void destroySet(std::set<uint32_t>* set)
    {
        return delete set;
    }

    std::map<uint32_t, ComponentAndMaterialDesc> id_to_component_and_material_desc_map_;

    uint32_t component_and_material_desc_id_initial_guess_ = 1;
    std::set<uint32_t> component_and_material_desc_ids_;
};
