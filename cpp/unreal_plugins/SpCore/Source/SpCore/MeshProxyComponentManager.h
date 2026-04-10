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
#include <Engine/EngineBaseTypes.h>    // ELevelTick
#include <Materials/MaterialInstanceDynamic.h>
#include <Materials/MaterialInterface.h>
#include <Math/Color.h>
#include <UObject/StrongObjectPtr.h>   // TStrongObjectPtr
#include <UObject/UObjectGlobals.h>    // LoadObject

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/ProxyComponentManager.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

class USceneComponent;
struct FActorComponentTickFunction;

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

//
// MeshProxyComponentManager inherits ProxyComponentManager<MeshProxyComponentManager, MeshProxyComponentData>
// (CRTP) and owns per-material-slot proxy geometry bookkeeping. Derived classes (e.g.,
// ObjectIdsMeshProxyComponentManager) override the virtual methods to customize material creation.
//

class MeshProxyComponentManager : public ProxyComponentManager<MeshProxyComponentManager, MeshProxyComponentData>
{
public:
    MeshProxyComponentManager() = delete;
    MeshProxyComponentManager(AActor* owner)
    {
        owner_ = owner;
    }

    //
    // Accessing proxy geometry. This function is called from an owning AActor when returning proxy geometry
    // to user Python code.
    //

    const std::map<uint32_t, MeshProxyGeometryDesc>& getMeshProxyGeometryDescs() const { return id_to_mesh_proxy_geometry_desc_map_; }

    //
    // ProxyComponentManager<MeshProxyComponentManager, MeshProxyComponentData> CRTP interface. These
    // functions are called from the ProxyComponentManager<...> base class.
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

        if (shouldProxyComponentBeHiddenInViewport(component)) {
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
            UMaterialInterface* material = createMaterialForProxyComponentMaterialSlot(mesh_proxy_geometry_desc_id, component, component->GetMaterial(i));
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

        if (shouldProxyComponentBeHiddenInViewport(component)) {
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
            UMaterialInterface* material = createMaterialForProxyComponentMaterialSlot(mesh_proxy_geometry_desc_id, component, component->GetMaterial(i));
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
    // Virtual methods for derived classes to override. These functions are called internally in registerProxyComponent(...)
    //

    virtual bool shouldProxyComponentBeHiddenInViewport(USceneComponent* component) const = 0;
    virtual UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* component, UMaterialInterface* material) = 0;

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

    AActor* owner_ = nullptr;

    std::map<uint32_t, MeshProxyGeometryDesc> id_to_mesh_proxy_geometry_desc_map_;
    std::set<uint32_t> mesh_proxy_geometry_desc_ids_;
    uint32_t mesh_proxy_geometry_desc_id_initial_guess_ = 1;
};


class ObjectIdsMeshProxyComponentManager : public MeshProxyComponentManager
{
public:
    ObjectIdsMeshProxyComponentManager(AActor* owner) : MeshProxyComponentManager(owner) {}

    void initialize()
    {
        emissive_material_ = TStrongObjectPtr<UMaterialInterface>(LoadObject<UMaterialInterface>(nullptr, Unreal::toTCharPtr("/SpContent/Materials/M_Emissive.M_Emissive")));
        SP_ASSERT(emissive_material_.Get());
    }

    void terminate()
    {
        emissive_material_.Reset();
    }

    UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* component, UMaterialInterface* material) override
    {
        SP_ASSERT(emissive_material_.Get());
        UMaterialInstanceDynamic* new_material = UMaterialInstanceDynamic::Create(emissive_material_.Get(), getOwner());
        SP_ASSERT(new_material);
        FLinearColor color = getLinearColorImpl(id);
        new_material->SetVectorParameterValue("EmissiveColor", color);
        return new_material;
    }

    bool shouldProxyComponentBeHiddenInViewport(USceneComponent* component) const override { return true; }

private:
    TStrongObjectPtr<UMaterialInterface> emissive_material_;

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


class ObjectIdsVisualizerMeshProxyComponentManager : public MeshProxyComponentManager
{
public:
    ObjectIdsVisualizerMeshProxyComponentManager(AActor* owner) : MeshProxyComponentManager(owner) {}

    void initialize()
    {
        diffuse_material_ = TStrongObjectPtr<UMaterialInterface>(LoadObject<UMaterialInterface>(nullptr, Unreal::toTCharPtr("/SpContent/Materials/M_Diffuse.M_Diffuse")));
        SP_ASSERT(diffuse_material_.Get());
    }

    void terminate()
    {
        diffuse_material_.Reset();
    }

    UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* component, UMaterialInterface* material) override
    {
        SP_ASSERT(diffuse_material_.Get());
        UMaterialInstanceDynamic* new_material = UMaterialInstanceDynamic::Create(diffuse_material_.Get(), getOwner());
        SP_ASSERT(new_material);
        FColor color = FColor::MakeRandomColor();
        new_material->SetVectorParameterValue("BaseColor", color);
        return new_material;
    }

    bool shouldProxyComponentBeHiddenInViewport(USceneComponent* component) const override { return false; }

private:
    TStrongObjectPtr<UMaterialInterface> diffuse_material_;
};
