//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // uint32_t

#include <algorithm>  // std::min
#include <map>
#include <ranges>     // std::views::filter, std::views::transform
#include <set>
#include <string>
#include <vector>

#include <Containers/UnrealString.h> // FString
#include <Containers/Array.h> 
#include <Engine/EngineTypes.h>      // EEndPlayReason
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // uint32
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpProxyComponentManager.generated.h"

class UActorComponent;
class UWorld;

template <typename TProxyComponentRegistry, typename TComponent>
concept CProxyComponentRegistryCanRegister =
    CComponent<TComponent> &&
    requires (TProxyComponentRegistry proxy_component_registry, TComponent* component) {
        { proxy_component_registry.registerProxyComponent(component, component) } -> std::same_as<void*>;
    };

template <typename TProxyComponentRegistry>
concept CProxyComponentRegistryCanUnregister =
    requires (TProxyComponentRegistry proxy_component_registry) {
        { proxy_component_registry.unregisterProxyComponent(nullptr) } -> std::same_as<void>;
    };

template <typename TProxyComponentRegistry, typename TComponent>
concept CProxyComponentRegistryShouldRegister =
    CComponent<TComponent> &&
    requires (TProxyComponentRegistry proxy_component_registry, TComponent* component) {
        { proxy_component_registry.shouldRegisterProxyComponent(component) } -> std::same_as<bool>;
    };

UCLASS(ClassGroup="SPEAR", Config=Spear, HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpProxyComponentManager : public AActor
{
    GENERATED_BODY()

private:
    struct ProxyComponentDesc
    {
        uint32_t id_ = 0;
        std::string name_;
        UActorComponent* proxy_component_ = nullptr;
        UActorComponent* component_ = nullptr;
        void* user_data_ = nullptr;
    };

public: 
    ASpProxyComponentManager();
    ~ASpProxyComponentManager() override;

    // AActor interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;
    void Tick(float delta_time) override;
    bool ShouldTickIfViewportsOnly() const override;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void Initialize();
    UFUNCTION(CallInEditor, Category="SPEAR")
    void Terminate();
    UFUNCTION(CallInEditor, Category="SPEAR")
    void Update();

    UFUNCTION(Category="SPEAR")
    bool IsInitialized() const;

protected:
    virtual void initialize();
    virtual void terminate();
    virtual void update();
    virtual void findAndDestroyAllProxyComponents(const std::vector<AActor*>& actors) {};
    virtual void findAndRegisterAllProxyComponents(const std::vector<AActor*>& actors) {};
    virtual void findAndUnregisterAllProxyComponents(const std::vector<AActor*>& actors) {};
    virtual void unregisterProxyComponents(const std::vector<std::string>& component_names) {};

    template <CComponent TComponent>
    void findAndDestroyProxyComponentsImpl(const std::vector<AActor*>& actors)
    {
        // Find all proxy components of type TComponent that belong to this manager.
        std::vector<TComponent*> proxy_components = Std::toVector<TComponent*>(
            getComponents<TComponent>(actors) |
            std::views::filter([](auto component) { return Std::contains(Unreal::toStdString(component->GetName()), "SP_PROXY_COMPONENT"); }) |
            std::views::filter([this](auto component) { return Std::contains(Unreal::toStdString(component->GetName()), getManagerName()); }));

        // Destroy proxy components.
        for (auto proxy_component : proxy_components) {
            SP_LOG("Destroying component: ", getLongComponentName(GetWorld(), proxy_component));
            destroyComponent(proxy_component);
        }
    }

    template <typename TComponent, typename TProxyComponentRegistry> requires
        CProxyComponentRegistryCanUnregister<TProxyComponentRegistry> && CProxyComponentRegistryCanRegister<TProxyComponentRegistry, TComponent>
    void findAndRegisterProxyComponentsImpl(const std::vector<AActor*>& actors, TProxyComponentRegistry* proxy_component_registry)
    {
        // Find all non-proxy components of type TComponent.
        std::map<std::string, TComponent*> non_proxy_components = Std::toMap<std::string, TComponent*>(
            getComponents<TComponent>(actors) |
            std::views::filter([](auto component) { return !Std::contains(Unreal::toStdString(component->GetName()), "SP_PROXY_COMPONENT"); }) |
            std::views::transform([this](auto component) { return std::make_pair(getLongComponentName(GetWorld(), component), component); }));

        // Find components to register, i.e., present in the world, of type TComponent, but not registered.
        std::map<std::string, TComponent*> components_to_register = Std::toMap<std::string, TComponent*>(
            non_proxy_components |
            std::views::filter([this](auto& pair) { auto& [name, component] = pair; return !Std::containsKey(name_to_proxy_component_desc_map_, name); }));

        // Register components.
        registerComponentsImpl<TComponent>(components_to_register, proxy_component_registry);
    }

    template <typename TComponent, typename TProxyComponentRegistry> requires
        CProxyComponentRegistryCanUnregister<TProxyComponentRegistry> && CProxyComponentRegistryCanRegister<TProxyComponentRegistry, TComponent>
    void findAndUnregisterProxyComponentsImpl(const std::vector<AActor*>& actors, TProxyComponentRegistry* proxy_component_registry)
    {
        // Find all non-proxy components of type TComponent.
        std::map<std::string, TComponent*> non_proxy_components = Std::toMap<std::string, TComponent*>(
            getComponents<TComponent>(actors) |
            std::views::filter([](auto component) { return !Std::contains(Unreal::toStdString(component->GetName()), "SP_PROXY_COMPONENT"); }) |
            std::views::transform([this](auto component) { return std::make_pair(getLongComponentName(GetWorld(), component), component); }));

        // Find components to unregister, i.e., registered and of type TComponent but not present in the world.
        std::vector<std::string> proxy_components_to_unregister = Std::toVector<std::string>(
            name_to_proxy_component_desc_map_ |
            std::views::filter([&non_proxy_components](auto& pair) { auto& [name, proxy_component_desc] = pair; return proxy_component_desc.proxy_component_->IsA(TComponent::StaticClass()); }) |
            std::views::filter([&non_proxy_components](auto& pair) { auto& [name, proxy_component_desc] = pair; return !Std::containsKey(non_proxy_components, name); }) |
            std::views::transform([this](auto& pair) { auto& [name, proxy_component_desc] = pair; return name; }));

        // Unregister components. Do this before registering so we can recycle IDs from the unregistered
        // components.
        unregisterProxyComponentsImpl(proxy_components_to_unregister, proxy_component_registry);
    }

    template <typename TComponent, typename TProxyComponentRegistry> requires
        CProxyComponentRegistryCanRegister<TProxyComponentRegistry, TComponent>
    void registerComponentsImpl(const std::map<std::string, TComponent*>& components, TProxyComponentRegistry* proxy_component_registry)
    {
        for (auto& [name, component] : components) {
            if (shouldRegisterComponent(proxy_component_registry, component)) {
                SP_LOG("Registering component: ", name);

                uint32_t proxy_component_desc_id = getId(ProxyComponentDescIdInitialGuess, proxy_component_desc_ids_);
                ProxyComponentDescIdInitialGuess = proxy_component_desc_id + 1;

                std::string proxy_component_name = getProxyComponentName(proxy_component_desc_id);

                SP_ASSERT(!proxy_component_desc_ids_.contains(proxy_component_desc_id));
                SP_ASSERT(!Std::containsKey(name_to_proxy_component_desc_map_, name));

                TComponent* proxy_component = createComponent<TComponent>(this, component, proxy_component_name);
                SP_ASSERT(proxy_component);

                SP_ASSERT(proxy_component_registry);
                void* user_data = proxy_component_registry->registerProxyComponent(proxy_component, component);

                ProxyComponentDesc proxy_component_desc;
                proxy_component_desc.id_ = proxy_component_desc_id;
                proxy_component_desc.name_ = name;
                proxy_component_desc.proxy_component_ = proxy_component;
                proxy_component_desc.component_ = component;
                proxy_component_desc.user_data_ = user_data;

                proxy_component_desc_ids_.insert(proxy_component_desc_id);
                Std::insert(name_to_proxy_component_desc_map_, name, std::move(proxy_component_desc));

                RegisteredProxyComponentDescIds.Add(proxy_component_desc_id);
                RegisteredProxyComponentDescNames.Add(Unreal::toFString(name));
            }
        }
    }

    template <typename TProxyComponentRegistry> requires
        CProxyComponentRegistryCanUnregister<TProxyComponentRegistry>
    void unregisterProxyComponentsImpl(const std::vector<std::string>& component_names, TProxyComponentRegistry* proxy_component_registry)
    {
        for (auto& name : component_names) {
            SP_LOG("Unregistering component: ", name);

            ProxyComponentDesc proxy_component_desc = name_to_proxy_component_desc_map_.at(name);
            uint32_t proxy_component_desc_id = proxy_component_desc.id_;

            SP_ASSERT(proxy_component_desc_ids_.contains(proxy_component_desc_id));
            SP_ASSERT(Std::containsKey(name_to_proxy_component_desc_map_, name));

            SP_ASSERT(proxy_component_registry);
            proxy_component_registry->unregisterProxyComponent(proxy_component_desc.user_data_);

            destroyComponent(proxy_component_desc.proxy_component_);

            Std::remove(proxy_component_desc_ids_, proxy_component_desc_id);
            Std::remove(name_to_proxy_component_desc_map_, name);

            RegisteredProxyComponentDescIds.Remove(proxy_component_desc_id);
            RegisteredProxyComponentDescNames.Remove(Unreal::toFString(name));
        }
    }

private:
    template <typename TComponent, typename TProxyComponentRegistry> requires
        CProxyComponentRegistryShouldRegister<TProxyComponentRegistry, TComponent>
    static bool shouldRegisterComponent(TProxyComponentRegistry proxy_component_registry, TComponent* component)
    {
        return proxy_component_registry->shouldRegisterComponent(component);
    }

    template <typename TComponent, typename TProxyComponentRegistry>
    static bool shouldRegisterComponent(TProxyComponentRegistry proxy_component_registry, TComponent* component)
    {
        return true;
    }

protected:
    static uint32_t getId(uint32_t initial_guess_id = 1, const std::set<uint32_t>& already_allocated_ids = {}, uint32_t max_id = 0x00ffffff);

private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR");
    bool bIsInitialized = false;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    uint32 ProxyComponentDescIdInitialGuess = 1;

    UPROPERTY(VisibleAnywhere, Category="SPEAR");
    TArray<uint32> RegisteredProxyComponentDescIds;

    UPROPERTY(VisibleAnywhere, Category="SPEAR");
    TArray<FString> RegisteredProxyComponentDescNames;

    std::string getManagerName() const;
    std::string getProxyComponentName(uint32_t id) const;

    template <CComponent TComponent>
    static std::vector<TComponent*> getComponents(const std::vector<AActor*>& actors)
    {
        std::vector<TComponent*> components;
        for (auto actor : actors) {
            SP_ASSERT(actor);
            std::vector<TComponent*> components_for_actor = Unreal::getComponentsByType<TComponent>(actor);
            components.insert(components.end(), components_for_actor.begin(), components_for_actor.end());
        }
        return components;
    }

    template <CNonSceneComponent TNonSceneComponent>
    static TNonSceneComponent* createComponent(UObject* owner, TNonSceneComponent* component, const std::string& name)
    {
        return Unreal::createComponentOutsideOwnerConstructor<TNonSceneComponent>(owner, name);
    }

    template <CSceneComponent TSceneComponent>
    static TSceneComponent* createComponent(UObject* owner, TSceneComponent* component, const std::string& name)
    {
        return Unreal::createSceneComponentOutsideOwnerConstructor<TSceneComponent>(owner, component, name);
    }

    static void destroyComponent(UActorComponent* component)
    {
        Unreal::destroyComponentOutsideOwnerConstructor(component);
    }

    static std::string getLongComponentName(const UWorld* world, const UActorComponent* component);

    bool request_reinitialize_ = false;
    bool is_initialized_ = false;

    std::map<std::string, ProxyComponentDesc> name_to_proxy_component_desc_map_;
    std::set<uint32_t> proxy_component_desc_ids_;
};
