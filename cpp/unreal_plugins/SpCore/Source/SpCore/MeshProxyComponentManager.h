//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint32_t

#include <map>
#include <set>
#include <string>

#include <Components/SkeletalMeshComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick
#include <Materials/MaterialInterface.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/ProxyComponentManager.h"
#include "SpCore/Std.h"
#include "SpCore/UnrealUtils.h"

class USceneComponent;
struct FActorComponentTickFunction;

//
// IMeshProxyComponentManager is a pure virtual interface that MeshProxyComponentManager calls
// to decide what materials to assign to proxy geometry.
//

struct IMeshProxyComponentManager
{
    virtual bool shouldProxyComponentBeHiddenInViewport(USceneComponent* component) const = 0;
    virtual UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* component, UMaterialInterface* material) = 0;
};

//
// MeshProxyComponentManager inherits ProxyComponentManager<MeshProxyComponentManager> (CRTP) and owns
// per-material-slot proxy geometry bookkeeping. It takes an IMeshProxyComponentManager* at construction
// for callbacks that vary per modality.
//

struct MeshProxyComponentData
{
    std::set<uint32_t> mesh_proxy_geometry_desc_ids_;
};

struct MeshProxyGeometryDesc
{
    uint32_t id_ = 0;
    bool is_valid_ = false;
    USceneComponent* component_ = nullptr;
    UMaterialInterface* material_ = nullptr;
};

class MeshProxyComponentManager : public ProxyComponentManager<MeshProxyComponentManager, MeshProxyComponentData>
{
public:
    MeshProxyComponentManager() = delete;

    MeshProxyComponentManager(IMeshProxyComponentManager* mesh_proxy_component_manager, AActor* owner)
    {
        mesh_proxy_component_manager_ = mesh_proxy_component_manager;
        owner_ = owner;
    }

    //
    // ProxyComponentManager<MeshProxyComponentManager> CRTP interface
    //

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

    bool shouldUnregisterProxyComponent(USceneComponent* proxy_component, MeshProxyComponentData* mesh_proxy_component_data) { return false; }

    MeshProxyComponentData* registerProxyComponent(UStaticMeshComponent* proxy_component, UStaticMeshComponent* component)
    {
        MeshProxyComponentData* mesh_proxy_component_data = createMeshProxyComponentData();

        bool new_visibility;
        bool propagate_to_children;

        // set visibility to false while configuring our proxy component
        new_visibility = false;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        // configure component
        proxy_component->SetStaticMesh(component->GetStaticMesh());

        if (mesh_proxy_component_manager_->shouldProxyComponentBeHiddenInViewport(component)) {
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
            uint32_t mesh_proxy_geometry_desc_id = registerMeshProxyGeometryImpl(component, component->GetMaterial(i));
            SP_LOG("Registering proxy geometry: ", UnrealUtils::getStableName(component), " (material slot = ", i, ", ID = ", mesh_proxy_geometry_desc_id, ")");
            UMaterialInterface* material = mesh_proxy_component_manager_->createMaterialForProxyComponentMaterialSlot(mesh_proxy_geometry_desc_id, component, component->GetMaterial(i));
            SP_ASSERT(material);
            proxy_component->SetMaterial(i, material);
            Std::insert(mesh_proxy_component_data->mesh_proxy_geometry_desc_ids_, mesh_proxy_geometry_desc_id);
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

        return mesh_proxy_component_data;
    }

    MeshProxyComponentData* registerProxyComponent(USkeletalMeshComponent* proxy_component, USkeletalMeshComponent* component)
    {
        MeshProxyComponentData* mesh_proxy_component_data = createMeshProxyComponentData();

        bool new_visibility;
        bool propagate_to_children;

        // set visibility to false while configuring our proxy component
        new_visibility = false;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        // configure component
        proxy_component->SetSkeletalMesh(component->GetSkeletalMeshAsset());
        proxy_component->SetLeaderPoseComponent(component);

        if (mesh_proxy_component_manager_->shouldProxyComponentBeHiddenInViewport(component)) {
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
            uint32_t mesh_proxy_geometry_desc_id = registerMeshProxyGeometryImpl(component, component->GetMaterial(i));
            SP_LOG("Registering proxy geometry: ", UnrealUtils::getStableName(component), " (material slot = ", i, ", ID = ", mesh_proxy_geometry_desc_id, ")");
            UMaterialInterface* material = mesh_proxy_component_manager_->createMaterialForProxyComponentMaterialSlot(mesh_proxy_geometry_desc_id, component, component->GetMaterial(i));
            SP_ASSERT(material);
            proxy_component->SetMaterial(i, material);
            Std::insert(mesh_proxy_component_data->mesh_proxy_geometry_desc_ids_, mesh_proxy_geometry_desc_id);
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

        return mesh_proxy_component_data;
    }

    void invalidateProxyComponent(MeshProxyComponentData* mesh_proxy_component_data)
    {
        SP_ASSERT(mesh_proxy_component_data);

        for (auto mesh_proxy_geometry_desc_id : mesh_proxy_component_data->mesh_proxy_geometry_desc_ids_) {
            SP_LOG("Invalidating proxy geometry: (ID = ", mesh_proxy_geometry_desc_id, ")");
            invalidateMeshProxyGeometryImpl(mesh_proxy_geometry_desc_id);
        }
    }

    void unregisterProxyComponent(MeshProxyComponentData* mesh_proxy_component_data)
    {
        SP_ASSERT(mesh_proxy_component_data);

        for (auto mesh_proxy_geometry_desc_id : mesh_proxy_component_data->mesh_proxy_geometry_desc_ids_) {
            SP_LOG("Unregistering proxy geometry: (ID = ", mesh_proxy_geometry_desc_id, ")");
            unregisterMeshProxyGeometryImpl(mesh_proxy_geometry_desc_id);
        }

        destroyMeshProxyComponentData(mesh_proxy_component_data);
    }

    UWorld* getWorld() { return owner_->GetWorld(); }
    AActor* getOwner() { return owner_; }

    //
    // Geometry accessors
    //

    const std::map<uint32_t, MeshProxyGeometryDesc>& getMeshProxyGeometryDescs() const { return id_to_mesh_proxy_geometry_desc_map_; }

private:
    uint32_t registerMeshProxyGeometryImpl(USceneComponent* component, UMaterialInterface* material)
    {
        uint32_t mesh_proxy_geometry_desc_id = getId(mesh_proxy_geometry_desc_id_initial_guess_, mesh_proxy_geometry_desc_ids_);
        mesh_proxy_geometry_desc_id_initial_guess_ = mesh_proxy_geometry_desc_id + 1;

        MeshProxyGeometryDesc mesh_proxy_geometry_desc;
        mesh_proxy_geometry_desc.id_ = mesh_proxy_geometry_desc_id;
        mesh_proxy_geometry_desc.is_valid_ = true;
        mesh_proxy_geometry_desc.component_ = component;
        mesh_proxy_geometry_desc.material_ = material;

        Std::insert(mesh_proxy_geometry_desc_ids_, mesh_proxy_geometry_desc_id);
        Std::insert(id_to_mesh_proxy_geometry_desc_map_, mesh_proxy_geometry_desc_id, mesh_proxy_geometry_desc);

        return mesh_proxy_geometry_desc_id;
    }

    void invalidateMeshProxyGeometryImpl(uint32_t mesh_proxy_geometry_desc_id)
    {
        MeshProxyGeometryDesc& mesh_proxy_geometry_desc = id_to_mesh_proxy_geometry_desc_map_.at(mesh_proxy_geometry_desc_id);
        mesh_proxy_geometry_desc.is_valid_ = false;
        mesh_proxy_geometry_desc.component_ = nullptr;
        mesh_proxy_geometry_desc.material_ = nullptr;
    }

    void unregisterMeshProxyGeometryImpl(uint32_t mesh_proxy_geometry_desc_id)
    {
        Std::remove(mesh_proxy_geometry_desc_ids_, mesh_proxy_geometry_desc_id);
        Std::remove(id_to_mesh_proxy_geometry_desc_map_, mesh_proxy_geometry_desc_id);
    }

    static MeshProxyComponentData* createMeshProxyComponentData()
    {
        return new MeshProxyComponentData();
    }

    static void destroyMeshProxyComponentData(MeshProxyComponentData* mesh_proxy_component_data)
    {
        SP_ASSERT(mesh_proxy_component_data);
        delete mesh_proxy_component_data;
    }

    IMeshProxyComponentManager* mesh_proxy_component_manager_ = nullptr;
    AActor* owner_ = nullptr;

    std::map<uint32_t, MeshProxyGeometryDesc> id_to_mesh_proxy_geometry_desc_map_;
    std::set<uint32_t> mesh_proxy_geometry_desc_ids_;
    uint32_t mesh_proxy_geometry_desc_id_initial_guess_ = 1;
};
