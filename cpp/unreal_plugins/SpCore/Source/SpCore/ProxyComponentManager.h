//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint32_t

#include <map>
#include <ranges> // std::views::filter, std::views::transform
#include <set>
#include <string>
#include <vector>

#include <boost/predef.h> // BOOST_COMP_CLANG, BOOST_COMP_MSVC

#if BOOST_COMP_MSVC
    #include <format>
#endif

#include <Components/ActorComponent.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <UObject/ObjectMacros.h>           // RF_Transient
#include <UObject/WeakObjectPtrTemplates.h> // TWeakObjectPtr

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

class AActor;
class UActorComponent;
class UClass;
class UWorld;

//
// The proxy component system creates a "proxy" component for each eligible "non-proxy" component in a world (e.g.,
// each visible static mesh component), and keeps the proxy components in sync as non-proxy components are created,
// destroyed, shown, and hidden. Each proxy component can be rendered differently from its non-proxy component, e.g.,
// ASpObjectIdsProxyComponentManager renders each proxy component with materials whose colors encode IDs, so a scene
// capture can render a segmentation image.
//
// The system is split into two layers. The UE layer consists of the following actors:
//
//     ASpProxyComponentManager
//         Handles the actor lifecycle (Initialize(), Terminate(), Update(), Rebuild(), ticking, and PIE transitions).
//
//     ASpMeshProxyComponentManager
//         Chooses which component types get proxies (UStaticMeshComponent, USkeletalMeshComponent), dispatches into
//         a MeshProxyComponentManager for each type, and exposes mesh-specific functionality to Python.
//
//     ASpObjectIdsProxyComponentManager, ASpObjectIdsVisualizerProxyComponentManager
//         Choose a proxy strategy by owning a concrete MeshProxyComponentManager subclass.
//
// The plain C++ layer consists of the following classes:
//
//     ProxyComponentManager<...>
//         Keeps track of which non-proxy components have proxy components, allocates IDs, and delays unregistering
//         proxy components until rendered images no longer refer to them. This bookkeeping is independent of the
//         component type, so it is written once here.
//
//     MeshProxyComponentManager
//         Decides which mesh components are eligible, and creates and configures mesh proxy components.
//
//     ObjectIdsMeshProxyComponentManager, ObjectIdsVisualizerMeshProxyComponentManager
//         Choose the material for each proxy component material slot.
//
// Keeping UE plumbing (UCLASS, UFUNCTION, ticking, PIE) in the actors and proxy logic in plain C++ classes keeps
// each layer focused, and lets the proxy logic use templates, which UCLASSes can't. ProxyComponentManager<...> uses
// CRTP rather than virtual functions so it can call derived class functions that are templated on the component type.
//
// ProxyComponentManager<TDerivedProxyComponentManager, TDerivedProxyComponentData> is a CRTP base class that
// manages the lifecycle of proxy components. It is not possible to enforce this contract using a concept in
// our current CRTP design, so we can only enforce using a comment.
//
// TDerivedProxyComponentManager must provide:
//     bool shouldRegisterProxyForComponent(TComponent* non_proxy_component)
//     bool shouldUnregisterProxyForComponent(TComponent* non_proxy_component, TDerivedProxyComponentData* derived_proxy_component_data)
//     TProxyComponent* createProxyComponent(TComponent* non_proxy_component, const std::string& proxy_component_name)
//     TDerivedProxyComponentData registerProxyComponent(TProxyComponent* proxy_component, TComponent* non_proxy_component)
//     void invalidateProxyComponent(TDerivedProxyComponentData* derived_proxy_component_data)
//     void destroyProxyComponent(UActorComponent* proxy_component)
//     void unregisterProxyComponent(TDerivedProxyComponentData* derived_proxy_component_data)
//     UWorld* getWorld()
//     AActor* getOwner()
//
// ProxyComponentManagerBase is a non-template base class for ProxyComponentManager<...>, so that
// ASpProxyComponentManager::getProxyComponentManager() can return a common type, and ASpProxyComponentManager can call
// initialize() and terminate() through it.
//

class ProxyComponentManagerBase
{
public:
    virtual ~ProxyComponentManagerBase() = default;

    // Called from ASpProxyComponentManager::initializeImpl() and ASpProxyComponentManager::terminateImpl(). Derived
    // classes that override initialize() and terminate() should call the base class implementations, first in
    // initialize() and last in terminate().
    virtual void initialize() {}
    virtual void terminate() {}

    // Proxy component names begin with this prefix, so they can be distinguished from non-proxy components.
    virtual std::string getProxyComponentNamePrefix() const { return "__SP_PROXY_COMPONENT__"; }
};

template <typename TDerivedProxyComponentManager, typename TDerivedProxyComponentData>
class ProxyComponentManager : public ProxyComponentManagerBase
{
public:

    struct ProxyComponentDesc
    {
        uint32_t id_ = 0;
        std::string non_proxy_name_;
        TWeakObjectPtr<UActorComponent> proxy_component_;
        TWeakObjectPtr<UActorComponent> non_proxy_component_;
        TDerivedProxyComponentData derived_proxy_component_data_;
        bool is_valid_ = false;
        int unregister_frame_counter_ = -1;
    };

    //
    // ProxyComponentManagerBase interface. Called from ASpProxyComponentManager::initializeImpl() and
    // ASpProxyComponentManager::terminateImpl().
    //

    void initialize() override
    {
        ProxyComponentManagerBase::initialize();

        SP_ASSERT(non_proxy_name_to_proxy_component_desc_maps_.empty());
        SP_ASSERT(invalid_id_to_proxy_component_desc_map_.empty());
        SP_ASSERT(proxy_component_desc_ids_.empty());
    }

    void terminate() override
    {
        SP_ASSERT(non_proxy_name_to_proxy_component_desc_maps_.empty());
        SP_ASSERT(invalid_id_to_proxy_component_desc_map_.empty());
        SP_ASSERT(proxy_component_desc_ids_.empty());

        proxy_component_desc_id_initial_guess_ = 1;
        unregister_delay_frames_ = s_default_unregister_delay_frames_;

        ProxyComponentManagerBase::terminate();
    }

    //
    // Rendering can be buffered, so a rendered image might contain proxy IDs from components that have already been
    // destroyed. So we delay unregistering proxy components for unregister_delay_frames_ updates, so their IDs remain
    // valid for the images rendered while they were alive. If rendering is single-buffered, this can be set to 0.
    // Called from derived actor classes (e.g., ASpMeshProxyComponentManager::GetUnregisterDelayFrames() and
    // ASpMeshProxyComponentManager::SetUnregisterDelayFrames(...)).
    //

    int getUnregisterDelayFrames() const { return unregister_delay_frames_; }
    void setUnregisterDelayFrames(int value) { SP_ASSERT(value >= 0); unregister_delay_frames_ = value; }

    //
    // Proxy component lifecycle functions. Called from the ASpProxyComponentManager *ForAllTypes() hooks, which derived
    // actor classes (e.g., ASpMeshProxyComponentManager) implement by calling these functions for each component type.
    // initializeProxyComponentDescs<TComponent>() and terminateProxyComponentDescs<TComponent>() add and remove
    // bookkeeping for a component type. The request*() functions incrementally create, register, invalidate, destroy,
    // and unregister proxy components to keep them in sync with the non-proxy components in the world.
    // unregisterAndDestroyProxyComponents() destroys and unregisters all proxy components immediately.
    //

    // Called from ASpProxyComponentManager::initializeProxyComponentDescsForAllTypes()
    template <CComponent TComponent>
    void initializeProxyComponentDescs()
    {
        SP_ASSERT(!Std::containsKey(non_proxy_name_to_proxy_component_desc_maps_, TComponent::StaticClass()));
        Std::insert(non_proxy_name_to_proxy_component_desc_maps_, TComponent::StaticClass(), std::map<std::string, ProxyComponentDesc>());
    }

    // Called from ASpProxyComponentManager::terminateProxyComponentDescsForAllTypes()
    template <CComponent TComponent>
    void terminateProxyComponentDescs()
    {
        SP_ASSERT(Std::containsKey(non_proxy_name_to_proxy_component_desc_maps_, TComponent::StaticClass()));
        SP_ASSERT(non_proxy_name_to_proxy_component_desc_maps_.at(TComponent::StaticClass()).empty());
        Std::remove(non_proxy_name_to_proxy_component_desc_maps_, TComponent::StaticClass());
    }

    // Called from ASpProxyComponentManager::requestCreateAndRegisterProxyComponentsForAllTypes(...)
    template <CComponent TComponent>
    void requestCreateAndRegisterProxyComponents(const std::vector<UActorComponent*>& non_proxy_components)
    {
        // We know from the call site that all components are valid, of type TComponent, and not proxy components
        SP_ASSERT(Std::all(
            non_proxy_components |
            std::views::transform([this](auto non_proxy_component) { return Unreal::isValid(non_proxy_component) && non_proxy_component->IsA(TComponent::StaticClass()) && !Std::contains(Unreal::getUnrealName(non_proxy_component), getProxyComponentNamePrefix()); })));

        // We know from the call site that we have ProxyComponentDescs for the type TComponent
        SP_ASSERT(Std::containsKey(non_proxy_name_to_proxy_component_desc_maps_, TComponent::StaticClass()));

        // Get ProxyComponentDescs for type TComponent
        std::map<std::string, ProxyComponentDesc>& non_proxy_name_to_proxy_component_desc_map = non_proxy_name_to_proxy_component_desc_maps_.at(TComponent::StaticClass());

        // Find non-proxy components to create and register proxies for: present in the world, of type TComponent, TDerivedProxyComponentManager thinks it should be registered, not yet registered.
        std::map<std::string, TComponent*> non_proxy_components_to_create_and_register_proxies_for = Std::toMap<std::string, TComponent*>(
            non_proxy_components |
            std::views::transform([](auto non_proxy_component) { return static_cast<TComponent*>(non_proxy_component); }) |
            std::views::filter([this](auto non_proxy_component) { return getPtr()->shouldRegisterProxyForComponent(non_proxy_component); }) |
            std::views::transform([this](auto non_proxy_component) { return std::make_pair(getLongComponentName(getPtr()->getWorld(), non_proxy_component), non_proxy_component); }) |
            std::views::filter([&non_proxy_name_to_proxy_component_desc_map](const auto& pair) { const auto& [non_proxy_name, non_proxy_component] = pair; return !Std::containsKey(non_proxy_name_to_proxy_component_desc_map, non_proxy_name); }));

        // Create and register proxy components. This adds their descs to non_proxy_name_to_proxy_component_desc_maps_.
        createAndRegisterProxiesForComponentsImpl(non_proxy_components_to_create_and_register_proxies_for);
    }

    // Called from ASpProxyComponentManager::requestUnregisterAndDestroyProxyComponentsForAllTypes(...)
    void requestDecrementAndUnregisterProxyComponents()
    {
        // Find invalid proxy components.
        std::vector<uint32_t> invalid_proxy_component_desc_ids = Std::toVector<uint32_t>(
            invalid_id_to_proxy_component_desc_map_ |
            std::views::transform([](auto& pair) { auto& [id, proxy_component_desc] = pair; return id; }));

        // Decrement counters for invalid proxy components.
        decrementFrameCountersForInvalidProxyComponentsImpl(invalid_proxy_component_desc_ids);

        // Find invalid proxy components whose countdown has reached zero.
        std::vector<uint32_t> proxy_component_desc_ids_to_unregister = Std::toVector<uint32_t>(
            invalid_id_to_proxy_component_desc_map_ |
            std::views::filter([](auto& pair) { auto& [id, proxy_component_desc] = pair; return proxy_component_desc.unregister_frame_counter_ == 0; }) |
            std::views::transform([](auto& pair) { auto& [id, proxy_component_desc] = pair; return id; }));

        // Unregister proxy components. This removes their descs from invalid_id_to_proxy_component_desc_map_.
        unregisterProxyComponentsImpl(proxy_component_desc_ids_to_unregister);
    }

    // Called from ASpProxyComponentManager::requestUnregisterAndDestroyProxyComponentsForAllTypes(...)
    template <CComponent TComponent>
    void requestUnregisterAndDestroyProxyComponents(const std::vector<UActorComponent*>& non_proxy_components)
    {
        // We know from the call site that all components are valid, of type TComponent, and not proxy components
        SP_ASSERT(Std::all(
            non_proxy_components |
            std::views::transform([this](auto non_proxy_component) { return Unreal::isValid(non_proxy_component) && non_proxy_component->IsA(TComponent::StaticClass()) && !Std::contains(Unreal::getUnrealName(non_proxy_component), getProxyComponentNamePrefix()); })));

        // We know from the call site that we have ProxyComponentDescs for the type TComponent
        SP_ASSERT(Std::containsKey(non_proxy_name_to_proxy_component_desc_maps_, TComponent::StaticClass()));

        // Find all non-proxy components of type TComponent present in the world.
        std::map<std::string, TComponent*> non_proxy_components_in_world = Std::toMap<std::string, TComponent*>(
            non_proxy_components |
            std::views::transform([](auto non_proxy_component) { return static_cast<TComponent*>(non_proxy_component); }) |
            std::views::transform([this](auto non_proxy_component) { return std::make_pair(getLongComponentName(getPtr()->getWorld(), non_proxy_component), non_proxy_component); }));

        // Get ProxyComponentDescs for type TComponent
        std::map<std::string, ProxyComponentDesc>& non_proxy_name_to_proxy_component_desc_map = non_proxy_name_to_proxy_component_desc_maps_.at(TComponent::StaticClass());

        // Find non-proxy component names to invalidate and destroy by iterating over registered non-proxy components
        std::vector<std::string> non_proxy_component_names_to_invalidate_and_destroy;
        for (auto& [non_proxy_name, desc] : non_proxy_name_to_proxy_component_desc_map) {
            SP_ASSERT(Unreal::isValid(desc.proxy_component_)); // we know by construction that each proxy component is valid

            // if the non-proxy component is missing from the world, then invalidate
            if (!Std::containsKey(non_proxy_components_in_world, non_proxy_name)) {
                non_proxy_component_names_to_invalidate_and_destroy.push_back(non_proxy_name);
            } else {
                UActorComponent* non_proxy_component = Unreal::toPtr(desc.non_proxy_component_);

                // if the non-proxy is invalid, then invalidate
                if (!non_proxy_component) {
                    non_proxy_component_names_to_invalidate_and_destroy.push_back(non_proxy_name);
                } else {
                    SP_ASSERT(non_proxy_component->IsA(TComponent::StaticClass())); // if a non-proxy component is valid, then it must be of type TComponent because we static_cast below

                    // if TDerivedProxyComponentManager thinks it should be unregistered, then invalidate
                    if (getPtr()->shouldUnregisterProxyForComponent(static_cast<TComponent*>(non_proxy_component), &desc.derived_proxy_component_data_)) {
                        non_proxy_component_names_to_invalidate_and_destroy.push_back(non_proxy_name);
                    }
                }
            }
        }

        // Invalidate and destroy proxy components. This moves their descs from
        // non_proxy_name_to_proxy_component_desc_maps_ to invalid_id_to_proxy_component_desc_map_.
        invalidateAndDestroyProxiesForComponentsImpl(non_proxy_name_to_proxy_component_desc_map, non_proxy_component_names_to_invalidate_and_destroy);
    }

    // Called from ASpProxyComponentManager::unregisterAndDestroyProxyComponentsForAllTypes()
    void unregisterAndDestroyProxyComponents()
    {
        for (auto& [component_class, non_proxy_name_to_proxy_component_desc_map] : non_proxy_name_to_proxy_component_desc_maps_) {

            // Find non-proxy component names to invalidate and destroy: registered.
            std::vector<std::string> non_proxy_component_names_to_invalidate_and_destroy = Std::toVector<std::string>(
                non_proxy_name_to_proxy_component_desc_map |
                std::views::transform([](auto& pair) { auto& [non_proxy_name, desc] = pair; return non_proxy_name; }));

            // Invalidate and destroy proxy components. This moves their descs from
            // non_proxy_name_to_proxy_component_desc_maps_ to invalid_id_to_proxy_component_desc_map_.
            invalidateAndDestroyProxiesForComponentsImpl(non_proxy_name_to_proxy_component_desc_map, non_proxy_component_names_to_invalidate_and_destroy);

            // All proxy components of this type should be invalid at this point.
            SP_ASSERT(non_proxy_name_to_proxy_component_desc_map.empty());
        }

        // Find proxy components to unregister: invalid.
        std::vector<uint32_t> proxy_component_desc_ids_to_unregister = Std::toVector<uint32_t>(
            invalid_id_to_proxy_component_desc_map_ |
            std::views::transform([](auto& pair) { auto& [id, proxy_component_desc] = pair; return id; }));

        // Unregister proxy components. This removes their descs from invalid_id_to_proxy_component_desc_map_.
        unregisterProxyComponentsImpl(proxy_component_desc_ids_to_unregister);

        // All proxy components should be unregistered at this point.
        SP_ASSERT(invalid_id_to_proxy_component_desc_map_.empty());
        SP_ASSERT(proxy_component_desc_ids_.empty());
    }

private:

    //
    // Helper functions for the lifecycle functions above. Each function takes non-proxy components or proxy component
    // desc IDs that have already been selected by the caller, performs one lifecycle step on each of them by calling
    // into TDerivedProxyComponentManager through getPtr(), and updates our bookkeeping.
    //

    template <CComponent TComponent>
    void createAndRegisterProxiesForComponentsImpl(const std::map<std::string, TComponent*>& non_proxy_components)
    {
        std::map<std::string, ProxyComponentDesc>& non_proxy_name_to_proxy_component_desc_map = non_proxy_name_to_proxy_component_desc_maps_.at(TComponent::StaticClass());

        for (auto& [non_proxy_name, non_proxy_component] : non_proxy_components) {
            if (Config::get<bool>("SP_CORE.PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Creating proxy component for non-proxy component: ", non_proxy_name);
            }

            uint32_t proxy_component_desc_id = getId(proxy_component_desc_id_initial_guess_, proxy_component_desc_ids_);
            proxy_component_desc_id_initial_guess_ = proxy_component_desc_id + 1;

            std::string proxy_component_name = getProxyComponentName(proxy_component_desc_id);

            SP_ASSERT(!proxy_component_desc_ids_.contains(proxy_component_desc_id));
            SP_ASSERT(!Std::containsKey(non_proxy_name_to_proxy_component_desc_map, non_proxy_name));

            // TDerivedProxyComponentManager creates the proxy component, so it can choose the proxy component's type.
            // We use auto here so we can pass the proxy component back to TDerivedProxyComponentManager with the type
            // it chose. We require the proxy component to be owned by getOwner(), and to be named
            // proxy_component_name, so we can distinguish it from non-proxy components. We also require it to be
            // transient, so it isn't saved into a map, and isn't duplicated from the editor world into a PIE world, see
            // AActor::Serialize(...) in Engine/Source/Runtime/Engine/Private/Actor.cpp.

            auto proxy_component = getPtr()->createProxyComponent(non_proxy_component, proxy_component_name);
            SP_ASSERT(proxy_component);
            SP_ASSERT(proxy_component->GetOwner() == getPtr()->getOwner());
            SP_ASSERT(Unreal::isNamed(proxy_component, proxy_component_name));
            SP_ASSERT(proxy_component->HasAnyFlags(RF_Transient));

            if (Config::get<bool>("SP_CORE.PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Registering proxy component for non-proxy component: ", non_proxy_name);
            }

            ProxyComponentDesc proxy_component_desc;
            proxy_component_desc.id_ = proxy_component_desc_id;
            proxy_component_desc.non_proxy_name_ = non_proxy_name;
            proxy_component_desc.is_valid_ = true;
            proxy_component_desc.proxy_component_ = proxy_component;
            proxy_component_desc.non_proxy_component_ = non_proxy_component;
            proxy_component_desc.derived_proxy_component_data_ = getPtr()->registerProxyComponent(proxy_component, non_proxy_component);

            Std::insert(proxy_component_desc_ids_, proxy_component_desc_id);
            Std::insert(non_proxy_name_to_proxy_component_desc_map, non_proxy_name, std::move(proxy_component_desc));
        }
    }

    void decrementFrameCountersForInvalidProxyComponentsImpl(const std::vector<uint32_t>& proxy_component_desc_ids)
    {
        for (auto id : proxy_component_desc_ids) {
            ProxyComponentDesc& proxy_component_desc = invalid_id_to_proxy_component_desc_map_.at(id);
            SP_ASSERT(!proxy_component_desc.is_valid_);
            SP_ASSERT(proxy_component_desc.unregister_frame_counter_ >= 1);
            proxy_component_desc.unregister_frame_counter_--;
        }
    }

    void invalidateAndDestroyProxiesForComponentsImpl(std::map<std::string, ProxyComponentDesc>& non_proxy_name_to_proxy_component_desc_map, const std::vector<std::string>& non_proxy_component_names)
    {
        for (auto& non_proxy_name : non_proxy_component_names) {
            SP_ASSERT(Std::containsKey(non_proxy_name_to_proxy_component_desc_map, non_proxy_name));

            // We move the proxy component desc out of non_proxy_name_to_proxy_component_desc_maps_ and into
            // invalid_id_to_proxy_component_desc_map_. This way, if a component is removed and then re-added (e.g.,
            // if it is hidden and then shown again), a new proxy component can be created for it immediately,
            // without needing to wait for the old proxy component to be unregistered.
            ProxyComponentDesc proxy_component_desc = std::move(non_proxy_name_to_proxy_component_desc_map.at(non_proxy_name));
            SP_ASSERT(proxy_component_desc.is_valid_);
            Std::remove(non_proxy_name_to_proxy_component_desc_map, non_proxy_name);

            if (Config::get<bool>("SP_CORE.PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Invalidating proxy component for non-proxy component: ", non_proxy_name);
            }
            proxy_component_desc.is_valid_ = false;
            proxy_component_desc.unregister_frame_counter_ = unregister_delay_frames_;
            proxy_component_desc.non_proxy_component_ = nullptr;

            getPtr()->invalidateProxyComponent(&proxy_component_desc.derived_proxy_component_data_);

            if (Config::get<bool>("SP_CORE.PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Destroying proxy component for non-proxy component: ", non_proxy_name);
            }
            UActorComponent* proxy_component = Unreal::toPtr(proxy_component_desc.proxy_component_);
            SP_ASSERT(proxy_component);
            getPtr()->destroyProxyComponent(proxy_component);
            SP_ASSERT(!Unreal::isValid(proxy_component_desc.proxy_component_));
            proxy_component_desc.proxy_component_ = nullptr;

            uint32_t proxy_component_desc_id = proxy_component_desc.id_;
            Std::insert(invalid_id_to_proxy_component_desc_map_, proxy_component_desc_id, std::move(proxy_component_desc));

            // If unregister_delay_frames_ is 0, then there is no need to delay unregistering, e.g., because rendering
            // is single-buffered, so we unregister immediately.
            if (unregister_delay_frames_ == 0) {
                unregisterProxyComponentsImpl({proxy_component_desc_id});
            }
        }
    }

    void unregisterProxyComponentsImpl(const std::vector<uint32_t>& proxy_component_desc_ids)
    {
        for (auto id : proxy_component_desc_ids) {
            SP_ASSERT(proxy_component_desc_ids_.contains(id));
            SP_ASSERT(Std::containsKey(invalid_id_to_proxy_component_desc_map_, id));

            ProxyComponentDesc proxy_component_desc = std::move(invalid_id_to_proxy_component_desc_map_.at(id));
            SP_ASSERT(!proxy_component_desc.is_valid_);

            if (Config::get<bool>("SP_CORE.PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Unregistering proxy component for non-proxy component: ", proxy_component_desc.non_proxy_name_);
            }

            getPtr()->unregisterProxyComponent(&proxy_component_desc.derived_proxy_component_data_);

            Std::remove(proxy_component_desc_ids_, id);
            Std::remove(invalid_id_to_proxy_component_desc_map_, id);
        }
    }

protected:

    //
    // Called from ProxyComponentManager<...> to allocate proxy component desc IDs, and from derived classes to allocate
    // their own IDs (e.g., MeshProxyComponentManager allocates an ID for each proxy component material slot).
    //

    static uint32_t getId(uint32_t initial_guess_id = 1, const std::set<uint32_t>& already_allocated_ids = {}, uint32_t max_id = 0x00ffffff)
    {
        SP_ASSERT(initial_guess_id >= 1);
        SP_ASSERT(max_id >= 1);

        for (uint32_t i = initial_guess_id; i <= max_id; i++) {
            if (!already_allocated_ids.contains(i)) {
                return i;
            }
        }

        for (uint32_t i = 1; i < initial_guess_id; i++) {
            if (!already_allocated_ids.contains(i)) {
                return i;
            }
        }

        SP_ASSERT(false);
        return 0;
    }

private:

    // Only TDerivedProxyComponentManager can construct this class. This guarantees that the static_cast in getPtr() is
    // valid, e.g., it prevents class B from mistakenly deriving from ProxyComponentManager<A, ...>.
    friend TDerivedProxyComponentManager;
    ProxyComponentManager() = default;

    std::string getManagerName() const
    {
        AActor* actor = getPtr()->getOwner();
        SP_ASSERT(actor);
        return Unreal::getUnrealName(actor);
    }

    std::string getProxyComponentName(uint32_t id) const
    {
        #if BOOST_COMP_MSVC
            return getProxyComponentNamePrefix() + getManagerName() + "_" + std::format("{:#018x}", id) + "__";
        #elif BOOST_COMP_CLANG
            return getProxyComponentNamePrefix() + getManagerName() + "_" + (boost::format("0x%016x")%id).str() + "__";
        #else
            #error
        #endif
    }

    static std::string getLongComponentName(const UWorld* world, const UActorComponent* component)
    {
        SP_ASSERT(world);
        SP_ASSERT(component);

        bool include_actor_stable_name = true;
        bool include_actor_unreal_name = true;
        std::string actor_and_component_name = UnrealUtils::getStableName(component, include_actor_stable_name, include_actor_unreal_name);

        return Unreal::toStdString(world->GetPathName()) + ":" + actor_and_component_name + ":" + Std::toString(component->GetUniqueID()) + ":" + Std::toStringFromPtr(component);
    }

    TDerivedProxyComponentManager* getPtr() { return static_cast<TDerivedProxyComponentManager*>(this); }
    const TDerivedProxyComponentManager* getPtr() const { return static_cast<const TDerivedProxyComponentManager*>(this); }

    std::map<UClass*, std::map<std::string, ProxyComponentDesc>> non_proxy_name_to_proxy_component_desc_maps_;
    std::map<uint32_t, ProxyComponentDesc> invalid_id_to_proxy_component_desc_map_;
    std::set<uint32_t> proxy_component_desc_ids_;
    uint32_t proxy_component_desc_id_initial_guess_ = 1;

    inline static constexpr int s_default_unregister_delay_frames_ = 3;
    int unregister_delay_frames_ = s_default_unregister_delay_frames_;
};
