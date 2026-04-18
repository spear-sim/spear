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
#include <Engine/CollisionProfile.h>        // UCollisionProfile
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Engine/EngineBaseTypes.h>         // ELevelTick
#include <Materials/MaterialInstanceDynamic.h>
#include <Materials/MaterialInterface.h>
#include <Math/Color.h>
#include <UObject/StrongObjectPtr.h>        // TStrongObjectPtr
#include <UObject/UObjectGlobals.h>         // LoadObject
#include <UObject/WeakObjectPtrTemplates.h> // TWeakObjectPtr

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
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
    TWeakObjectPtr<USceneComponent> component_;
    TWeakObjectPtr<UMaterialInterface> material_;
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

    virtual ~MeshProxyComponentManager() = default;

    //
    // Accessing proxy geometry. This function is called from an owning AActor when returning proxy geometry
    // to user Python code.
    //

    const std::map<uint32_t, MeshProxyGeometryDesc>& getMeshProxyGeometryDescs() const { return id_to_mesh_proxy_geometry_desc_map_; }

    //
    // Allow/ignore lists
    //

    const std::vector<AActor*>& getAllowedActors() const { return allowed_actors_; }
    void setAllowedActors(const std::vector<AActor*>& allowed_actors) { allowed_actors_ = allowed_actors; }

    const std::vector<USceneComponent*>& getAllowedComponents() const { return allowed_components_; }
    void setAllowedComponents(const std::vector<USceneComponent*>& allowed_components) { allowed_components_ = allowed_components; }

    const std::vector<AActor*>& getIgnoredActors() const { return ignored_actors_; }
    void setIgnoredActors(const std::vector<AActor*>& ignored_actors) { ignored_actors_ = ignored_actors; }

    const std::vector<USceneComponent*>& getIgnoredComponents() const { return ignored_components_; }
    void setIgnoredComponents(const std::vector<USceneComponent*>& ignored_components) { ignored_components_ = ignored_components; }

    //
    // ProxyComponentManager<MeshProxyComponentManager, MeshProxyComponentData> CRTP interface. These
    // functions are called from the ProxyComponentManager<...> base class.
    //

    bool shouldRegisterProxyForComponent(USceneComponent* component)
    {
        return isVisible(component) && isAllowed(component);
    }

    bool shouldUnregisterProxyForComponent(USceneComponent* component, MeshProxyComponentData* mesh_proxy_component_data)
    {
        return !isVisible(component) || !isAllowed(component);
    }

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
            proxy_component->bCastDynamicShadow = false;
            proxy_component->bCastStaticShadow = false;
            proxy_component->bAffectDistanceFieldLighting = false;
            proxy_component->bAffectDynamicIndirectLighting = false;
        }

        proxy_component->SetCanEverAffectNavigation(false);
        proxy_component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        proxy_component->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName); // needed in editor worlds
        proxy_component->SetGenerateOverlapEvents(false);

        for (int i = 0; i < component->GetNumMaterials(); i++) {
            uint32_t mesh_proxy_geometry_desc_id = registerMeshProxyGeometryImpl(component, component->GetMaterial(i));
            if (Config::isInitialized() && Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Registering proxy geometry: ", UnrealUtils::getStableName(component), " (material slot = ", i, ", ID = ", mesh_proxy_geometry_desc_id, ")");
            }
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
        proxy_component->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName); // needed in editor worlds
        proxy_component->SetGenerateOverlapEvents(false);

        for (int i = 0; i < component->GetNumMaterials(); i++) {
            uint32_t mesh_proxy_geometry_desc_id = registerMeshProxyGeometryImpl(component, component->GetMaterial(i));
            if (Config::isInitialized() && Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Registering proxy geometry: ", UnrealUtils::getStableName(component), " (material slot = ", i, ", ID = ", mesh_proxy_geometry_desc_id, ")");
            }
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
            if (Config::isInitialized() && Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Invalidating proxy geometry: (ID = ", mesh_proxy_geometry_desc_id, ")");
            }
            invalidateMeshProxyGeometryImpl(mesh_proxy_geometry_desc_id);
        }
    }

    void unregisterProxyComponent(MeshProxyComponentData* mesh_proxy_component_data)
    {
        SP_ASSERT(mesh_proxy_component_data);

        for (auto mesh_proxy_geometry_desc_id : mesh_proxy_component_data->mesh_proxy_geometry_desc_ids_) {
            if (Config::isInitialized() && Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Unregistering proxy geometry: (ID = ", mesh_proxy_geometry_desc_id, ")");
            }
            unregisterMeshProxyGeometryImpl(mesh_proxy_geometry_desc_id);
        }

        destroyMeshProxyComponentData(mesh_proxy_component_data);
    }

    UWorld* getWorld() { return owner_->GetWorld(); }
    AActor* getOwner() { return owner_; }

private:
    virtual bool shouldProxyComponentBeHiddenInViewport(USceneComponent* component) const = 0;
    virtual UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* component, UMaterialInterface* material) = 0;

    bool isVisible(USceneComponent* component)
    {
        SP_ASSERT(component);

        AActor* owner = component->GetOwner();
        if (!owner) {
            return false;
        }

        UWorld* world = component->GetWorld();
        if (!world) {
            return false;
        }

        if (world->IsGameWorld()) {
            if (owner->IsHidden()) {
                return false;
            } else {
                return component->IsVisible();
            }
        } else {
            #if WITH_EDITOR // defined in an auto-generated header
                if (owner->IsHiddenEd()) {
                    return false;
                } else if (owner->IsTemporarilyHiddenInEditor()) {
                    return false;
                } else {
                    return component->IsVisibleInEditor();
                }
            #else
                return false;
            #endif
        }
    }

    bool isAllowed(USceneComponent* component)
    {
        SP_ASSERT(component);

        // if either allow list is non-empty, the component must be explicitly allowed
        if (!allowed_actors_.empty() || !allowed_components_.empty()) {
            if (!Std::contains(allowed_components_, component) && !Std::contains(allowed_actors_, component->GetOwner())) {
                return false;
            }
        }

        // if either ignore list is non-empty, the component must not be ignored
        if (Std::contains(ignored_components_, component) || Std::contains(ignored_actors_, component->GetOwner())) {
            return false;
        }

        return true;
    }

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

    std::vector<AActor*> allowed_actors_;
    std::vector<USceneComponent*> allowed_components_;
    std::vector<AActor*> ignored_actors_;
    std::vector<USceneComponent*> ignored_components_;
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
