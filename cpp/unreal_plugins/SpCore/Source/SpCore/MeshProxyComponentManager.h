//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint32_t

#include <map>
#include <set>
#include <string>
#include <vector>

#include <Components/SkeletalMeshComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/CollisionProfile.h>        // UCollisionProfile
#include <Engine/EngineBaseTypes.h>         // ELevelTick
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Materials/MaterialInterface.h>
#include <Math/Color.h>
#include <UObject/Class.h>                  // UClass
#include <UObject/ObjectMacros.h>           // RF_Transient
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
    TWeakObjectPtr<USceneComponent> non_proxy_component_;
    TWeakObjectPtr<UMaterialInterface> non_proxy_material_;
};

//
// MeshProxyComponentManager inherits ProxyComponentManager<MeshProxyComponentManager, MeshProxyComponentData>
// (CRTP) and owns per-material-slot proxy geometry bookkeeping. Derived classes (e.g., ObjectIdsMeshProxyComponentManager)
// override the virtual methods to customize material creation.
//

class MeshProxyComponentManager : public ProxyComponentManager<MeshProxyComponentManager, MeshProxyComponentData>
{
public:
    MeshProxyComponentManager() = delete;
    MeshProxyComponentManager(AActor* owner)
    {
        SP_ASSERT(owner);
        owner_ = owner;
    }

    virtual ~MeshProxyComponentManager() = default;

    //
    // ProxyComponentManagerBase interface. Called from ASpProxyComponentManager::initializeImpl() and
    // ASpProxyComponentManager::terminateImpl().
    //

    void initialize() override
    {
        ProxyComponentManager::initialize();

        SP_ASSERT(id_to_mesh_proxy_geometry_desc_map_.empty());
        SP_ASSERT(mesh_proxy_geometry_desc_ids_.empty());
        SP_ASSERT(allowed_actors_.empty());
        SP_ASSERT(allowed_components_.empty());
        SP_ASSERT(ignored_actors_.empty());
        SP_ASSERT(ignored_components_.empty());
    }

    void terminate() override
    {
        SP_ASSERT(id_to_mesh_proxy_geometry_desc_map_.empty());
        SP_ASSERT(mesh_proxy_geometry_desc_ids_.empty());

        mesh_proxy_geometry_desc_id_initial_guess_ = 1;
        allowed_actors_.clear();
        allowed_components_.clear();
        ignored_actors_.clear();
        ignored_components_.clear();

        ProxyComponentManager::terminate();
    }

    //
    // MeshProxyComponentManager interface. Derived classes (e.g., ObjectIdsMeshProxyComponentManager) override these
    // functions to choose whether proxy components are hidden in the viewport, and which material to use for each
    // proxy component material slot. Called from registerProxyComponent(...).
    //

    virtual bool shouldProxyComponentBeHiddenInViewport(USceneComponent* non_proxy_component) const { SP_ASSERT(false); return false; }
    virtual UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* non_proxy_component, UMaterialInterface* non_proxy_material) { SP_ASSERT(false); return nullptr; }

    //
    // Called from ASpMeshProxyComponentManager, e.g., when returning proxy geometry descs, allow lists, and ignore lists
    // to Python.
    //

    const std::map<uint32_t, MeshProxyGeometryDesc>& getMeshProxyGeometryDescs() const { return id_to_mesh_proxy_geometry_desc_map_; }

    std::vector<AActor*> getAllowedActors() const
    {
        std::vector<AActor*> allowed_actors = Unreal::toPtr(allowed_actors_);
        SP_ASSERT(Unreal::isValid(allowed_actors));
        return allowed_actors;
    }

    void setAllowedActors(const std::vector<AActor*>& allowed_actors)
    {
        SP_ASSERT(Unreal::isValid(allowed_actors));
        allowed_actors_ = Std::toVector<TWeakObjectPtr<AActor>>(allowed_actors);
    }

    std::vector<USceneComponent*> getAllowedComponents() const
    {
        std::vector<USceneComponent*> allowed_components = Unreal::toPtr(allowed_components_);
        SP_ASSERT(Unreal::isValid(allowed_components));
        return allowed_components;
    }

    void setAllowedComponents(const std::vector<USceneComponent*>& allowed_components)
    {
        SP_ASSERT(Unreal::isValid(allowed_components));
        allowed_components_ = Std::toVector<TWeakObjectPtr<USceneComponent>>(allowed_components);
    }

    std::vector<AActor*> getIgnoredActors() const
    {
        std::vector<AActor*> ignored_actors = Unreal::toPtr(ignored_actors_);
        SP_ASSERT(Unreal::isValid(ignored_actors));
        return ignored_actors;
    }

    void setIgnoredActors(const std::vector<AActor*>& ignored_actors)
    {
        SP_ASSERT(Unreal::isValid(ignored_actors));
        ignored_actors_ = Std::toVector<TWeakObjectPtr<AActor>>(ignored_actors);
    }

    std::vector<USceneComponent*> getIgnoredComponents() const
    {
        std::vector<USceneComponent*> ignored_components = Unreal::toPtr(ignored_components_);
        SP_ASSERT(Unreal::isValid(ignored_components));
        return ignored_components;
    }

    void setIgnoredComponents(const std::vector<USceneComponent*>& ignored_components)
    {
        SP_ASSERT(Unreal::isValid(ignored_components));
        ignored_components_ = Std::toVector<TWeakObjectPtr<USceneComponent>>(ignored_components);
    }

    //
    // ProxyComponentManager<...> CRTP interface. These functions are called from ProxyComponentManager<...>.
    //

    bool shouldRegisterProxyForComponent(USceneComponent* non_proxy_component) const
    {
        return isVisible(non_proxy_component) && isAllowed(non_proxy_component) && isCompatibleType(non_proxy_component);
    }

    bool shouldUnregisterProxyForComponent(USceneComponent* non_proxy_component, MeshProxyComponentData* mesh_proxy_component_data) const
    {
        return !shouldRegisterProxyForComponent(non_proxy_component);
    }

    template <CSceneComponent TSceneComponent>
    TSceneComponent* createProxyComponent(TSceneComponent* non_proxy_component, const std::string& proxy_component_name)
    {
        TSceneComponent* proxy_component = UnrealUtils::createSceneComponentOutsideOwnerConstructor<TSceneComponent>(getOwner(), non_proxy_component, proxy_component_name, RF_Transient);
        SP_ASSERT(proxy_component);
        return proxy_component;
    }

    MeshProxyComponentData registerProxyComponent(UStaticMeshComponent* proxy_component, UStaticMeshComponent* non_proxy_component)
    {
        MeshProxyComponentData mesh_proxy_component_data;

        bool new_visibility;
        bool propagate_to_children;

        // set visibility to false while configuring our proxy component
        new_visibility = false;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        // configure component
        proxy_component->SetStaticMesh(non_proxy_component->GetStaticMesh());

        if (shouldProxyComponentBeHiddenInViewport(non_proxy_component)) {
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

        for (int i = 0; i < non_proxy_component->GetNumMaterials(); i++) {
            uint32_t mesh_proxy_geometry_desc_id = registerMeshProxyGeometryImpl(non_proxy_component, non_proxy_component->GetMaterial(i));
            if (Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Registering proxy geometry for non-proxy component: ", UnrealUtils::getStableName(non_proxy_component), " (material slot = ", i, ", ID = ", mesh_proxy_geometry_desc_id, ")");
            }
            UMaterialInterface* material = createMaterialForProxyComponentMaterialSlot(mesh_proxy_geometry_desc_id, non_proxy_component, non_proxy_component->GetMaterial(i));
            SP_ASSERT(material);
            proxy_component->SetMaterial(i, material);
            Std::insert(mesh_proxy_component_data.mesh_proxy_geometry_desc_ids_, mesh_proxy_geometry_desc_id);
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

    MeshProxyComponentData registerProxyComponent(USkeletalMeshComponent* proxy_component, USkeletalMeshComponent* non_proxy_component)
    {
        MeshProxyComponentData mesh_proxy_component_data;

        bool new_visibility;
        bool propagate_to_children;

        // set visibility to false while configuring our proxy component
        new_visibility = false;
        propagate_to_children = true;
        proxy_component->SetVisibility(new_visibility, propagate_to_children);

        // configure component
        proxy_component->SetSkeletalMesh(non_proxy_component->GetSkeletalMeshAsset());
        proxy_component->SetLeaderPoseComponent(non_proxy_component);

        if (shouldProxyComponentBeHiddenInViewport(non_proxy_component)) {
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

        for (int i = 0; i < non_proxy_component->GetNumMaterials(); i++) {
            uint32_t mesh_proxy_geometry_desc_id = registerMeshProxyGeometryImpl(non_proxy_component, non_proxy_component->GetMaterial(i));
            if (Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Registering proxy geometry for non-proxy component: ", UnrealUtils::getStableName(non_proxy_component), " (material slot = ", i, ", ID = ", mesh_proxy_geometry_desc_id, ")");
            }
            UMaterialInterface* material = createMaterialForProxyComponentMaterialSlot(mesh_proxy_geometry_desc_id, non_proxy_component, non_proxy_component->GetMaterial(i));
            SP_ASSERT(material);
            proxy_component->SetMaterial(i, material);
            Std::insert(mesh_proxy_component_data.mesh_proxy_geometry_desc_ids_, mesh_proxy_geometry_desc_id);
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
            if (Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Invalidating proxy geometry (ID = ", mesh_proxy_geometry_desc_id, ")");
            }
            invalidateMeshProxyGeometryImpl(mesh_proxy_geometry_desc_id);
        }
    }

    void destroyProxyComponent(UActorComponent* proxy_component)
    {
        SP_ASSERT(proxy_component);
        UnrealUtils::destroyComponentOutsideOwnerConstructor(proxy_component);
    }

    void unregisterProxyComponent(MeshProxyComponentData* mesh_proxy_component_data)
    {
        SP_ASSERT(mesh_proxy_component_data);

        for (auto mesh_proxy_geometry_desc_id : mesh_proxy_component_data->mesh_proxy_geometry_desc_ids_) {
            if (Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Unregistering proxy geometry (ID = ", mesh_proxy_geometry_desc_id, ")");
            }
            unregisterMeshProxyGeometryImpl(mesh_proxy_geometry_desc_id);
        }
    }

    UWorld* getWorld() const
    {
        return getOwner()->GetWorld();
    }

    AActor* getOwner() const
    {
        AActor* owner = Unreal::toPtr(owner_);
        SP_ASSERT(owner);
        return owner;
    }

private:

    bool isVisible(USceneComponent* non_proxy_component) const
    {
        SP_ASSERT(non_proxy_component);

        AActor* owner = non_proxy_component->GetOwner();
        if (!owner) {
            return false;
        }

        UWorld* world = non_proxy_component->GetWorld();
        if (!world) {
            return false;
        }

        if (world->IsGameWorld()) {
            if (owner->IsHidden()) {
                return false;
            } else {
                return non_proxy_component->IsVisible();
            }
        } else {
            #if WITH_EDITOR // defined in an auto-generated header
                if (owner->IsHiddenEd()) {
                    return false;
                } else if (owner->IsTemporarilyHiddenInEditor()) {
                    return false;
                } else {
                    return non_proxy_component->IsVisibleInEditor();
                }
            #else
                return false;
            #endif
        }
    }

    bool isAllowed(USceneComponent* non_proxy_component) const
    {
        SP_ASSERT(non_proxy_component);

        TWeakObjectPtr<USceneComponent> non_proxy_component_ptr = non_proxy_component;
        TWeakObjectPtr<AActor> owner_ptr = non_proxy_component->GetOwner();

        // if either allow list is non-empty, the component must be explicitly allowed
        if (!allowed_actors_.empty() || !allowed_components_.empty()) {
            if (!Std::contains(allowed_components_, non_proxy_component_ptr) && !Std::contains(allowed_actors_, owner_ptr)) {
                return false;
            }
        }

        // if either ignore list is non-empty, the component must not be ignored
        if (Std::contains(ignored_components_, non_proxy_component_ptr) || Std::contains(ignored_actors_, owner_ptr)) {
            return false;
        }

        return true;
    }

    bool isCompatibleType(USceneComponent* non_proxy_component) const
    {
        // We only support exact class matches, because derived classes (e.g., instanced foliage) can store their
        // geometry in derived-only data structures that aren't visible to the base class, so creating a proxy for them
        // using the base class interface isn't meaningful.
        static std::set<UClass*> allowed_classes = {UStaticMeshComponent::StaticClass(), USkeletalMeshComponent::StaticClass()};
        return allowed_classes.contains(non_proxy_component->GetClass());
    }

    uint32_t registerMeshProxyGeometryImpl(USceneComponent* non_proxy_component, UMaterialInterface* non_proxy_material)
    {
        uint32_t mesh_proxy_geometry_desc_id = getId(mesh_proxy_geometry_desc_id_initial_guess_, mesh_proxy_geometry_desc_ids_);
        mesh_proxy_geometry_desc_id_initial_guess_ = mesh_proxy_geometry_desc_id + 1;

        MeshProxyGeometryDesc mesh_proxy_geometry_desc;
        mesh_proxy_geometry_desc.id_ = mesh_proxy_geometry_desc_id;
        mesh_proxy_geometry_desc.is_valid_ = true;
        mesh_proxy_geometry_desc.non_proxy_component_ = non_proxy_component;
        mesh_proxy_geometry_desc.non_proxy_material_ = non_proxy_material;

        Std::insert(mesh_proxy_geometry_desc_ids_, mesh_proxy_geometry_desc_id);
        Std::insert(id_to_mesh_proxy_geometry_desc_map_, mesh_proxy_geometry_desc_id, mesh_proxy_geometry_desc);

        return mesh_proxy_geometry_desc_id;
    }

    void invalidateMeshProxyGeometryImpl(uint32_t mesh_proxy_geometry_desc_id)
    {
        MeshProxyGeometryDesc& mesh_proxy_geometry_desc = id_to_mesh_proxy_geometry_desc_map_.at(mesh_proxy_geometry_desc_id);
        mesh_proxy_geometry_desc.is_valid_ = false;
        mesh_proxy_geometry_desc.non_proxy_component_ = nullptr;
        mesh_proxy_geometry_desc.non_proxy_material_ = nullptr;
    }

    void unregisterMeshProxyGeometryImpl(uint32_t mesh_proxy_geometry_desc_id)
    {
        Std::remove(mesh_proxy_geometry_desc_ids_, mesh_proxy_geometry_desc_id);
        Std::remove(id_to_mesh_proxy_geometry_desc_map_, mesh_proxy_geometry_desc_id);
    }

    TWeakObjectPtr<AActor> owner_;

    std::map<uint32_t, MeshProxyGeometryDesc> id_to_mesh_proxy_geometry_desc_map_;
    std::set<uint32_t> mesh_proxy_geometry_desc_ids_;
    uint32_t mesh_proxy_geometry_desc_id_initial_guess_ = 1;

    std::vector<TWeakObjectPtr<AActor>> allowed_actors_;
    std::vector<TWeakObjectPtr<USceneComponent>> allowed_components_;
    std::vector<TWeakObjectPtr<AActor>> ignored_actors_;
    std::vector<TWeakObjectPtr<USceneComponent>> ignored_components_;
};

//
// ObjectIdsMeshProxyComponentManager and ObjectIdsVisualizerMeshProxyComponentManager implement two proxy strategies by
// overriding the MeshProxyComponentManager interface. ObjectIdsMeshProxyComponentManager assigns each proxy component
// material slot an emissive material whose color encodes the slot's ID, and hides proxy components in the viewport, so
// they are only visible to scene captures. This way, a scene capture can render a segmentation image whose pixels can
// be mapped back to non-proxy components and materials via getMeshProxyGeometryDescs().
// ObjectIdsVisualizerMeshProxyComponentManager assigns each slot a diffuse material with a random color, and keeps proxy
// components visible in the viewport, so users can inspect the segmentation in the editor.
//

class ObjectIdsMeshProxyComponentManager : public MeshProxyComponentManager
{
public:
    ObjectIdsMeshProxyComponentManager(AActor* owner) : MeshProxyComponentManager(owner) {}

    // ProxyComponentManagerBase interface

    void initialize() override
    {
        MeshProxyComponentManager::initialize();
        emissive_material_ = TStrongObjectPtr<UMaterialInterface>(LoadObject<UMaterialInterface>(nullptr, Unreal::toTCharPtr("/SpContent/Materials/M_Emissive.M_Emissive")));
        SP_ASSERT(emissive_material_.Get());
    }

    void terminate() override
    {
        emissive_material_.Reset();
        MeshProxyComponentManager::terminate();
    }

    // MeshProxyComponentManager interface

    bool shouldProxyComponentBeHiddenInViewport(USceneComponent* non_proxy_component) const override { return true; }

    UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* non_proxy_component, UMaterialInterface* non_proxy_material) override
    {
        SP_ASSERT(emissive_material_.Get());
        UMaterialInstanceDynamic* new_material = UMaterialInstanceDynamic::Create(emissive_material_.Get(), getOwner());
        SP_ASSERT(new_material);
        FLinearColor color = getLinearColorImpl(id);
        new_material->SetVectorParameterValue("EmissiveColor", color);
        return new_material;
    }

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

    // ProxyComponentManagerBase interface

    void initialize() override
    {
        MeshProxyComponentManager::initialize();
        diffuse_material_ = TStrongObjectPtr<UMaterialInterface>(LoadObject<UMaterialInterface>(nullptr, Unreal::toTCharPtr("/SpContent/Materials/M_Diffuse.M_Diffuse")));
        SP_ASSERT(diffuse_material_.Get());
    }

    void terminate() override
    {
        diffuse_material_.Reset();
        MeshProxyComponentManager::terminate();
    }

    // MeshProxyComponentManager interface

    bool shouldProxyComponentBeHiddenInViewport(USceneComponent* non_proxy_component) const override { return false; }

    UMaterialInterface* createMaterialForProxyComponentMaterialSlot(uint32_t id, USceneComponent* non_proxy_component, UMaterialInterface* non_proxy_material) override
    {
        SP_ASSERT(diffuse_material_.Get());
        UMaterialInstanceDynamic* new_material = UMaterialInstanceDynamic::Create(diffuse_material_.Get(), getOwner());
        SP_ASSERT(new_material);
        FColor color = FColor::MakeRandomColor();
        new_material->SetVectorParameterValue("BaseColor", color);
        return new_material;
    }

private:
    TStrongObjectPtr<UMaterialInterface> diffuse_material_;
};
