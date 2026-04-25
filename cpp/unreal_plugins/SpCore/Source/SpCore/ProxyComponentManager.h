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

#include <Engine/World.h>
#include <GameFramework/Actor.h>

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

class AActor;
class UActorComponent;
class UWorld;

//
// ProxyComponentManager<TDerivedProxyComponentManager, TDerivedProxyComponentData> is a CRTP base class that
// manages the lifecycle of proxy components. It is not possible to enforce this contract using a concept in
// our current CRTP design, so we can only enforce using a comment.
//
// TDerivedProxyComponentManager must provide:
//   TDerivedProxyComponentData* registerProxyComponent(TComponent* proxy_component, TComponent* component)
//   void invalidateProxyComponent(TDerivedProxyComponentData* derived_proxy_component_data)
//   void unregisterProxyComponent(TDerivedProxyComponentData* derived_proxy_component_data)
//   bool shouldRegisterProxyForComponent(TComponent* component)
//   bool shouldUnregisterProxyForComponent(TComponent* component, TDerivedProxyComponentData* derived_proxy_component_data)
//   UWorld* getWorld()
//   AActor* getOwner()
//

template <typename TDerivedProxyComponentManager, typename TDerivedProxyComponentData>
class ProxyComponentManager
{
public:

    struct ProxyComponentDesc
    {
        uint32_t id_ = 0;
        std::string name_;
        UActorComponent* proxy_component_ = nullptr;
        UActorComponent* component_ = nullptr;
        TDerivedProxyComponentData* derived_proxy_component_data_ = nullptr;
        bool mark_as_unregistered_ = false;
        int unregister_frame_counter_ = -1;
    };

    int getUnregisterDelayFrames() const { return unregister_delay_frames_; }
    void setUnregisterDelayFrames(int value) { unregister_delay_frames_ = value; }

    //
    // Request-based functions
    //

    // Called from ASpProxyComponentManager::requestCreateAndRegisterProxyComponentsForAllTypes(...)
    template <CComponent TComponent>
    void requestCreateAndRegisterProxyComponents(const std::vector<AActor*>& actors)
    {
        // Find all non-proxy components of type TComponent.
        std::map<std::string, TComponent*> non_proxy_components = Std::toMap<std::string, TComponent*>(
            getComponents<TComponent>(actors) |
            std::views::filter([](auto component) { return !Std::contains(Unreal::toStdString(component->GetName()), "SP_PROXY_COMPONENT"); }) |
            std::views::transform([this](auto component) { return std::make_pair(getLongComponentName(getPtr()->getWorld(), component), component); }));

        // Find components to create and register: present in the world, of type TComponent, TDerivedProxyComponentManager thinks it should be registered, and not yet registered.
        std::map<std::string, TComponent*> components_to_create_and_register = Std::toMap<std::string, TComponent*>(
            non_proxy_components |
            std::views::filter([this](auto& pair) { auto& [name, component] = pair; return getPtr()->shouldRegisterProxyForComponent(component); }) |
            std::views::filter([this](auto& pair) { auto& [name, component] = pair; return !Std::containsKey(name_to_proxy_component_desc_map_, name); }));

        // Create and register proxy components.
        createAndRegisterProxyComponentsImpl(components_to_create_and_register);
    }

    // Called from ASpProxyComponentManager::requestUnregisterAndDestroyProxyComponentsForAllTypes(...)
    void requestDecrementAndUnregisterProxyComponents()
    {
        // Find marked proxy components.
        std::vector<std::string> marked_proxy_components = Std::toVector<std::string>(
            name_to_proxy_component_desc_map_ |
            std::views::filter([](auto& pair) { auto& [name, proxy_component_desc] = pair; return proxy_component_desc.mark_as_unregistered_; }) |
            std::views::transform([](auto& pair) { auto& [name, proxy_component_desc] = pair; return name; }));

        // Decrement counters for marked proxy components.
        decrementFrameCountersForMarkedProxyComponentsImpl(marked_proxy_components);

        // Find marked proxy components whose countdown has reached zero.
        std::vector<std::string> proxy_components_to_unregister = Std::toVector<std::string>(
            name_to_proxy_component_desc_map_ |
            std::views::filter([](auto& pair) { auto& [name, proxy_component_desc] = pair; return proxy_component_desc.mark_as_unregistered_ && proxy_component_desc.unregister_frame_counter_ == 0; }) |
            std::views::transform([](auto& pair) { auto& [name, proxy_component_desc] = pair; return name; }));

        // Unregister proxy components.
        unregisterProxyComponentsImpl(proxy_components_to_unregister);
    }

    // Called from ASpProxyComponentManager::requestUnregisterAndDestroyProxyComponentsForAllTypes(...)
    template <CComponent TComponent>
    void requestUnregisterAndDestroyProxyComponents(const std::vector<AActor*>& actors)
    {
        // In this function it is only safe to call proxy_component_desc.proxy_component_->IsA(TComponent::StaticClass())
        // once we know a proxy_component_desc hasn't been marked as unregistered. If it has been marked as
        // unregistered, then proxy_component_desc.proxy_component_ will be nullptr. So we always check
        // mark_as_unregistered_ first, and then check proxy_component_->IsA(...) afterwards.

        // Find all non-proxy components of type TComponent.
        std::map<std::string, TComponent*> non_proxy_components = Std::toMap<std::string, TComponent*>(
            getComponents<TComponent>(actors) |
            std::views::filter([](auto component) { return !Std::contains(Unreal::toStdString(component->GetName()), "SP_PROXY_COMPONENT"); }) |
            std::views::transform([this](auto component) { return std::make_pair(getLongComponentName(getPtr()->getWorld(), component), component); }));

        // Find proxy components to mark as unregistered and destroy: registered, not already marked, of type TComponent, and not present in the world.
        std::vector<std::string> proxy_components_to_mark_as_unregistered_and_destroy = Std::toVector<std::string>(
            name_to_proxy_component_desc_map_ |
            std::views::filter([](auto& pair) { auto& [name, proxy_component_desc] = pair; return !proxy_component_desc.mark_as_unregistered_; }) |
            std::views::filter([](auto& pair) { auto& [name, proxy_component_desc] = pair; return proxy_component_desc.proxy_component_->IsA(TComponent::StaticClass()); }) |
            std::views::filter([&non_proxy_components](auto& pair) { auto& [name, proxy_component_desc] = pair; return !Std::containsKey(non_proxy_components, name); }) |
            std::views::transform([](auto& pair) { auto& [name, proxy_component_desc] = pair; return name; }));

        // Mark as unregistered and destroy proxy components.
        markAsUnregisteredAndDestroyProxyComponentsImpl(proxy_components_to_mark_as_unregistered_and_destroy);

        // Find proxy components to mark as unregistered and destroy: registered, not already marked, of type TComponent, and TDerivedProxyComponentManager thinks it should be unregistered.
        proxy_components_to_mark_as_unregistered_and_destroy = Std::toVector<std::string>(
            name_to_proxy_component_desc_map_ |
            std::views::filter([](auto& pair) { auto& [name, proxy_component_desc] = pair; return !proxy_component_desc.mark_as_unregistered_; }) |
            std::views::filter([](auto& pair) { auto& [name, proxy_component_desc] = pair; return proxy_component_desc.proxy_component_->IsA(TComponent::StaticClass()); }) |
            std::views::filter([this](auto& pair) {
                auto& [name, proxy_component_desc] = pair;
                SP_ASSERT(proxy_component_desc.component_->IsA(TComponent::StaticClass()));
                return getPtr()->shouldUnregisterProxyForComponent(static_cast<TComponent*>(proxy_component_desc.component_), proxy_component_desc.derived_proxy_component_data_); }) |
            std::views::transform([](auto& pair) {
                auto& [name, proxy_component_desc] = pair;
                return name; }));

        // Mark as unregistered and destroy proxy components.
        markAsUnregisteredAndDestroyProxyComponentsImpl(proxy_components_to_mark_as_unregistered_and_destroy);
    }

    // Called from ASpProxyComponentManager::unregisterAndDestroyProxyComponentsForAllTypes(...)
    void unregisterAndDestroyProxyComponents()
    {
        // Find proxy components to mark as unregistered and destroy: registered, not already marked.
        std::vector<std::string> proxy_components_to_mark_as_unregistered_and_destroy = Std::toVector<std::string>(
            name_to_proxy_component_desc_map_ |
            std::views::filter([](auto& pair) { auto& [name, proxy_component_desc] = pair; return !proxy_component_desc.mark_as_unregistered_; }) |
            std::views::transform([](auto& pair) { auto& [name, proxy_component_desc] = pair; return name; }));

        // Mark as unregistered and destroy components.
        markAsUnregisteredAndDestroyProxyComponentsImpl(proxy_components_to_mark_as_unregistered_and_destroy);

        // All proxy components should be marked as unregistered at this point.
        SP_ASSERT(Std::all(name_to_proxy_component_desc_map_ | std::views::transform([](auto& pair) { auto& [name, proxy_component_desc] = pair; return proxy_component_desc.mark_as_unregistered_; })));

        // Find proxy components to unregister: registered, already marked
        std::vector<std::string> proxy_components_to_unregister = Std::toVector<std::string>(
            name_to_proxy_component_desc_map_ |
            std::views::transform([](auto& pair) { auto& [name, proxy_component_desc] = pair; return name; }));

        // Unregister proxy components.
        unregisterProxyComponentsImpl(proxy_components_to_unregister);
    }

    //
    // Accessors
    //

    const std::map<std::string, ProxyComponentDesc>& getProxyComponentDescs() const { return name_to_proxy_component_desc_map_; }

    //
    // Static helpers
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
    TDerivedProxyComponentManager* getPtr() { return static_cast<TDerivedProxyComponentManager*>(this); }

    template <CComponent TComponent>
    void createAndRegisterProxyComponentsImpl(const std::map<std::string, TComponent*>& components)
    {
        for (auto& [name, component] : components) {
            if (Config::isInitialized() && Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Creating component: ", name);
            }

            uint32_t proxy_component_desc_id = getId(proxy_component_desc_id_initial_guess_, proxy_component_desc_ids_);
            proxy_component_desc_id_initial_guess_ = proxy_component_desc_id + 1;

            std::string proxy_component_name = getProxyComponentName(proxy_component_desc_id);

            SP_ASSERT(!proxy_component_desc_ids_.contains(proxy_component_desc_id));
            SP_ASSERT(!Std::containsKey(name_to_proxy_component_desc_map_, name));

            TComponent* proxy_component = createComponent<TComponent>(getPtr()->getOwner(), component, proxy_component_name);
            SP_ASSERT(proxy_component);

            if (Config::isInitialized() && Config::get<bool>("SP_CORE.MESH_PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Registering component: ", name);
            }

            TDerivedProxyComponentData* derived_proxy_component_data = getPtr()->registerProxyComponent(proxy_component, component);

            ProxyComponentDesc proxy_component_desc;
            proxy_component_desc.id_ = proxy_component_desc_id;
            proxy_component_desc.name_ = name;
            proxy_component_desc.proxy_component_ = proxy_component;
            proxy_component_desc.component_ = component;
            proxy_component_desc.derived_proxy_component_data_ = derived_proxy_component_data;

            Std::insert(proxy_component_desc_ids_, proxy_component_desc_id);
            Std::insert(name_to_proxy_component_desc_map_, name, std::move(proxy_component_desc));
        }
    }

    void decrementFrameCountersForMarkedProxyComponentsImpl(const std::vector<std::string>& component_names)
    {
        for (auto& name : component_names) {
            ProxyComponentDesc& proxy_component_desc = name_to_proxy_component_desc_map_.at(name);
            SP_ASSERT(proxy_component_desc.mark_as_unregistered_);
            SP_ASSERT(proxy_component_desc.unregister_frame_counter_ >= 1);
            proxy_component_desc.unregister_frame_counter_--;
        }
    }

    void markAsUnregisteredAndDestroyProxyComponentsImpl(const std::vector<std::string>& component_names)
    {
        for (auto& name : component_names) {
            SP_ASSERT(Std::containsKey(name_to_proxy_component_desc_map_, name));

            ProxyComponentDesc& proxy_component_desc = name_to_proxy_component_desc_map_.at(name);
            SP_ASSERT(!proxy_component_desc.mark_as_unregistered_);

            if (Config::isInitialized() && Config::get<bool>("SP_CORE.PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Marking component as unregistered and invalidating: ", name);
            }
            proxy_component_desc.mark_as_unregistered_ = true;
            proxy_component_desc.unregister_frame_counter_ = unregister_delay_frames_;
            proxy_component_desc.component_ = nullptr;

            getPtr()->invalidateProxyComponent(proxy_component_desc.derived_proxy_component_data_);

            if (Config::isInitialized() && Config::get<bool>("SP_CORE.PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Destroying component: ", name);
            }
            destroyComponent(proxy_component_desc.proxy_component_);
            proxy_component_desc.proxy_component_ = nullptr;
        }
    }

    void unregisterProxyComponentsImpl(const std::vector<std::string>& component_names)
    {
        for (auto& name : component_names) {
            if (Config::isInitialized() && Config::get<bool>("SP_CORE.PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Unregistering component: ", name);
            }

            ProxyComponentDesc proxy_component_desc = name_to_proxy_component_desc_map_.at(name);
            uint32_t proxy_component_desc_id = proxy_component_desc.id_;

            SP_ASSERT(proxy_component_desc_ids_.contains(proxy_component_desc_id));
            SP_ASSERT(Std::containsKey(name_to_proxy_component_desc_map_, name));

            getPtr()->unregisterProxyComponent(proxy_component_desc.derived_proxy_component_data_);

            Std::remove(proxy_component_desc_ids_, proxy_component_desc_id);
            Std::remove(name_to_proxy_component_desc_map_, name);
        }
    }

    std::string getManagerName()
    {
        AActor* actor = getPtr()->getOwner();
        SP_ASSERT(actor);
        return Unreal::toStdString(actor->GetName());
    }

    std::string getProxyComponentName(uint32_t id)
    {
        #if BOOST_COMP_MSVC
            return "__SP_PROXY_COMPONENT_" + getManagerName() + "_" + std::format("{:#018x}", id) + "__";
        #elif BOOST_COMP_CLANG
            return "__SP_PROXY_COMPONENT_" + getManagerName() + "_" + (boost::format("0x%016x")%id).str() + "__";
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

    template <CComponent TComponent>
    static std::vector<TComponent*> getComponents(const std::vector<AActor*>& actors)
    {
        std::vector<TComponent*> components;
        for (auto actor : actors) {
            std::vector<TComponent*> components_for_actor = UnrealUtils::getComponentsByType<TComponent>(actor);
            components.insert(components.end(), components_for_actor.begin(), components_for_actor.end());
        }
        return components;
    }

    template <CNonSceneComponent TNonSceneComponent>
    static TNonSceneComponent* createComponent(UObject* owner, TNonSceneComponent* component, const std::string& name)
    {
        return UnrealUtils::createComponentOutsideOwnerConstructor<TNonSceneComponent>(owner, name);
    }

    template <CSceneComponent TSceneComponent>
    static TSceneComponent* createComponent(UObject* owner, TSceneComponent* component, const std::string& name)
    {
        return UnrealUtils::createSceneComponentOutsideOwnerConstructor<TSceneComponent>(owner, component, name);
    }

    template <CComponent TComponent>
    void destroyProxyComponentsImpl(const std::vector<TComponent*>& components)
    {
        for (auto component : components) {
            if (Config::isInitialized() && Config::get<bool>("SP_CORE.PROXY_COMPONENT_MANAGER.VERBOSE")) {
                SP_LOG("Destroying component: ", getLongComponentName(getPtr()->getWorld(), component));
            }
            destroyComponent(component);
        }
    }

    static void destroyComponent(UActorComponent* component)
    {
        UnrealUtils::destroyComponentOutsideOwnerConstructor(component);
    }

    std::map<std::string, ProxyComponentDesc> name_to_proxy_component_desc_map_;
    std::set<uint32_t> proxy_component_desc_ids_;
    uint32_t proxy_component_desc_id_initial_guess_ = 1;
    int unregister_delay_frames_ = 3;
};
