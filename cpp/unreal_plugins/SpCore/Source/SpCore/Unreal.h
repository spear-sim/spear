//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t

#include <concepts> // std::derived_from
#include <map>
#include <ranges>   // std::views::filter, std::views::transform
#include <string>
#include <utility>  // std::make_pair, std::pair
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>        // TArray
#include <Containers/UnrealString.h> // FString::operator*
#include <EngineUtils.h>             // TActorIterator
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // TCHAR
#include <UObject/Class.h>           // EIncludeSuperFlag
#include <UObject/NameTypes.h>       // FName
#include <UObject/Object.h>          // UObject
#include <UObject/UnrealType.h>      // FProperty

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

class UClass;
class UStruct;
class UWorld;

template <typename TComponent>
concept CComponent = std::derived_from<TComponent, UActorComponent>;

template <typename TSceneComponent>
concept CSceneComponent = CComponent<TSceneComponent> && std::derived_from<TSceneComponent, USceneComponent>;

template <typename TActor>
concept CActor = std::derived_from<TActor, AActor>;

class SPCORE_API Unreal
{
public:
    Unreal() = delete;
    ~Unreal() = delete;

    //
    // String functions
    //

    static std::string toStdString(const FString& str);
    static std::string toStdString(const FName& str);
    static std::string toStdString(const TCHAR* str);
    static FString toFString(const std::string& str);
    static FName toFName(const std::string& str);

    //
    // Container functions
    //

    template <typename TValue>
    static std::vector<TValue> toStdVector(const TArray<TValue>& src)
    {
        std::vector<TValue> dest;
        for (auto& data : src) {
            dest.push_back(data);
        }
        return dest;
    }

    template <typename TDestValue, typename TSrcValue>
    static std::vector<TDestValue> toStdVectorOf(const TArray<TSrcValue>& src)
    {
        std::vector<TDestValue> dest;
        for (auto& data : src) {
            dest.push_back(reinterpret_cast<TDestValue>(data));
        }
        return dest;
    }

    //
    // Helper functions to get actor and component names
    //

    static std::string getStableActorName(const AActor* actor);
    static void setStableActorName(const AActor* actor, const std::string& stable_name);

    #if WITH_EDITOR // defined in an auto-generated header
        static void requestUpdateStableActorName(const AActor* actor);
    #endif

    static std::string getStableComponentName(const CComponent auto* actor_component, bool include_stable_actor_name = false)
    {
        SP_ASSERT(actor_component);
        std::string component_name = toStdString(actor_component->GetName());
        if (include_stable_actor_name) {
            AActor* actor = actor_component->GetOwner();
            SP_ASSERT(actor);
            component_name = getStableActorName(actor) + ":" + component_name;
        }
        return component_name;
    }

    static std::string getStableComponentName(const CSceneComponent auto* scene_component, bool include_stable_actor_name = false)
    {
        SP_ASSERT(scene_component);
        std::string component_name = toStdString(scene_component->GetName());
        TArray<USceneComponent*> parents;
        scene_component->GetParentComponents(parents);
        for (auto parent : parents) {
            component_name = toStdString(parent->GetName()) + "." + component_name;
        }
        if (include_stable_actor_name) {
            AActor* actor = scene_component->GetOwner();
            SP_ASSERT(actor);
            component_name = getStableActorName(actor) + ":" + component_name;
        }
        return component_name;
    }

    //
    // Helper functions to create components.
    //
    // If we are inside the constructor of an owner AActor (either directly or indirectly via the constructor of a child
    // component), then we must call CreateDefaultSubobject.
    // 
    // If we are outside the constructor of an owner AActor (either directly or indirectly via the constructor of a child
    // component), then we must call NewObject and RegisterComponent.
    // 
    // If we are creating a USceneComponent that is intended to be a root component, then we must call SetRootComponent
    // to attach the newly created component to its parent AActor.
    //
    // If we are creating a USceneComponent that is not intended to be a root component, then its parent must also be a
    // USceneComponent, and we must call SetupAttachment to attach the newly created component to its parent component.
    //
    // SetupAttachment must be called before RegisterComponent.
    //

    template <CComponent TComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* createComponentInsideOwnerConstructor(AActor* owner, const std::string& name)
    {
        SP_ASSERT(owner);
        TReturnAsComponent* actor_component = owner->CreateDefaultSubobject<TComponent>(toFName(name));
        SP_ASSERT(actor_component);
        return actor_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentInsideOwnerConstructor(AActor* owner, const std::string& name)
    {
        SP_ASSERT(owner);
        TReturnAsSceneComponent* scene_component = owner->CreateDefaultSubobject<TSceneComponent>(toFName(name));
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        return scene_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentInsideOwnerConstructor(UObject* owner, USceneComponent* parent, const std::string& name)
    {
        SP_ASSERT(owner);
        TReturnAsSceneComponent* scene_component = owner->CreateDefaultSubobject<TSceneComponent>(toFName(name));
        SP_ASSERT(scene_component);
        scene_component->SetupAttachment(parent);
        return scene_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentInsideOwnerConstructor(USceneComponent* owner, const std::string& name)
    {
        return createComponentInsideOwnerConstructor<TSceneComponent, TReturnAsSceneComponent>(owner, owner, name);
    }

    template <CComponent TComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TComponent* createComponentOutsideOwnerConstructor(AActor* owner, const std::string& name)
    {
        SP_ASSERT(owner);
        TReturnAsComponent* actor_component = NewObject<TComponent>(owner, toFName(name));
        SP_ASSERT(actor_component);
        actor_component->RegisterComponent();
        return actor_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentOutsideOwnerConstructor(AActor* owner, const std::string& name)
    {
        SP_ASSERT(owner);
        TReturnAsSceneComponent* scene_component = NewObject<TSceneComponent>(owner, toFName(name));
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        scene_component->RegisterComponent();
        return scene_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentOutsideOwnerConstructor(UObject* owner, USceneComponent* parent, const std::string& name)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        TReturnAsSceneComponent* scene_component = NewObject<TSceneComponent>(owner, toFName(name));
        SP_ASSERT(scene_component);
        scene_component->SetupAttachment(parent);
        scene_component->RegisterComponent();
        return scene_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentOutsideOwnerConstructor(USceneComponent* owner, const std::string& name)
    {
        return createComponentOutsideOwnerConstructor<TSceneComponent, TReturnAsSceneComponent>(owner, owner, name);
    }

    // 
    // Find actors unconditionally and return an std::vector or an std::map
    //

    static std::vector<AActor*> findActors(const UWorld* world);
    static std::map<std::string, AActor*> findActorsAsMap(const UWorld* world);

    //
    // Find actors by name or tag or type and return an std::vector
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByName(const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        std::map<std::string, TReturnAsActor*> actor_map = findActorsByNameAsMap<TActor, TReturnAsActor>(world, names);
        std::vector<TReturnAsActor*> actors;
        if (return_null_if_not_found) {
            actors = Std::toVector<TReturnAsActor*>(
                names |
                std::views::transform([&actor_map](const auto& name) { return Std::containsKey(actor_map, name) ? actor_map.at(name) : nullptr; }));
        } else {
            actors = Std::toVector<TReturnAsActor*>(
                names |
                std::views::filter([&actor_map](const auto& name)    { return Std::containsKey(actor_map, name); }) |
                std::views::transform([&actor_map](const auto& name) { return actor_map.at(name); }));
        }
        return actors;
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTag(const UWorld* world, const std::string& tag)
    {
        return findActorsByTagAny<TActor, TReturnAsActor>(world, {tag});
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTagAny(const UWorld* world, const std::vector<std::string>& tags)
    {
        auto actors = Std::toVector<TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::any(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return actor; }));
        return actors;
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTagAll(const UWorld* world, const std::vector<std::string>& tags)
    {
        auto actors = Std::toVector<TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::all(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return actor; }));
        return actors;
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByType(const UWorld* world)
    {
        SP_ASSERT(world);
        std::vector<TReturnAsActor*> actors;
        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            TReturnAsActor* actor = *itr;
            SP_ASSERT(actor);
            actors.push_back(actor);
        }
        return actors;
    }

    //
    // Find actors by name or tag or type and return an std::map
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByNameAsMap(const UWorld* world, const std::vector<std::string>& names)
    {
        auto actors = Std::toMap<std::string, TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::filter([](auto actor)       { return getActorHasStableName(actor); }) |
            std::views::filter([&names](auto actor) { return Std::contains(names, getStableActorName(actor)); }) |
            std::views::transform([](auto actor)    { return std::make_pair(getStableActorName(actor), actor); }));
        return actors;
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAsMap(const UWorld* world, const std::string& tag)
    {
        return findActorsByTagAnyAsMap<TActor, TReturnAsActor>(world, {tag});
    }
    
    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAnyAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        auto actors = Std::toMap<std::string, TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::filter([](auto actor)          { return getActorHasStableName(actor); }) |
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::any(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return std::make_pair(getStableActorName(actor), actor); }));
        return actors;
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAllAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        auto actors = Std::toMap<std::string, TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::filter([](auto actor)          { return getActorHasStableName(actor); }) |
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::all(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return std::make_pair(getStableActorName(actor), actor); }));
        return actors;
    }
    
    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTypeAsMap(const UWorld* world)
    {
        auto actors = Std::toMap<std::string, TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::filter([](auto actor)    { return getActorHasStableName(actor); }) |
            std::views::transform([](auto actor) { return std::make_pair(getStableActorName(actor), actor); }));
        return actors;
    }

    //
    // Find actor by name or tag or type and return a TActor* pointer
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByName(const UWorld* world, const std::string& name, bool assert_if_not_found = true)
    {
        bool assert_if_multiple_found = true;
        bool return_null_if_not_found = false;
        return getItem(findActorsByName<TActor, TReturnAsActor>(
            world, {name}, return_null_if_not_found), reinterpret_cast<TReturnAsActor*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTag(const UWorld* world, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(findActorsByTag<TActor, TReturnAsActor>(world, tag), reinterpret_cast<TReturnAsActor*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTagAny(const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(findActorsByTagAny<TActor, TReturnAsActor>(world, tags), reinterpret_cast<TReturnAsActor*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTagAll(const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(findActorsByTagAll<TActor, TReturnAsActor>(world, tags), reinterpret_cast<TReturnAsActor*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByType(const UWorld* world, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(findActorsByType<TActor, TReturnAsActor>(world), reinterpret_cast<TReturnAsActor*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    //
    // Get components unconditionally and return an std::vector or an std::map
    //

    std::vector<UActorComponent*> getComponents(const AActor* actor);
    std::map<std::string, UActorComponent*> getComponentsAsMap(const AActor* actor);

    //
    // Get components by name or tag or type and return an std::vector
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByName(const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        std::map<std::string, TReturnAsComponent*> component_map = getComponentsByNameAsMap<TComponent, TReturnAsComponent>(actor, names);
        std::vector<TReturnAsComponent*> components;
        if (return_null_if_not_found) {
            components = Std::toVector<TReturnAsComponent*>(
                names |
                std::views::transform([&component_map](const auto& name) { return Std::containsKey(component_map, name) ? component_map.at(name) : nullptr; }));
        } else {
            components = Std::toVector<TReturnAsComponent*>(
                names |
                std::views::filter([&component_map](const auto& name)    { return Std::containsKey(component_map, name); }) |
                std::views::transform([&component_map](const auto& name) { return component_map.at(name); }));
        }
        return components;
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTag(const AActor* actor, const std::string& tag)
    {
        return getComponentsByTagAny<TComponent, TReturnAsComponent>(actor, {tag});
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagAny(const AActor* actor, const std::vector<std::string>& tags)
    {
        auto components = Std::toVector<TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor) |
            std::views::transform([&tags](auto component) { return std::make_pair(component, getComponentHasTags(component, tags)); }) |
            std::views::filter([](const auto& pair)       { const auto& [component, has_tags] = pair; return Std::any(has_tags); }) |
            std::views::transform([](const auto& pair)    { const auto& [component, has_tags] = pair; return component; }));
        return components;
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagAll(const AActor* actor, const std::vector<std::string>& tags)
    {
        auto components = Std::toVector<TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor) |
            std::views::transform([&tags](auto component) { return std::make_pair(component, getComponentHasTags(component, tags)); }) |
            std::views::filter([](const auto& pair)       { const auto& [component, has_tags] = pair; return Std::all(has_tags); }) |
            std::views::transform([](const auto& pair)    { const auto& [component, has_tags] = pair; return component; }));
        return components;
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByType(const AActor* actor)
    {
        TArray<TComponent*> components;
        actor->GetComponents<TComponent>(components);
        return toStdVectorOf<TReturnAsComponent*>(components);
    }

    //
    // Get components by name or tag or type and return an std::map
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByNameAsMap(const AActor* actor, const std::vector<std::string>& names)
    {
        auto components = Std::toMap<std::string, TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor) |
            std::views::filter([&names](auto component) { return Std::contains(names, getStableComponentName(component)); }) |
            std::views::transform([](auto component)    { return std::make_pair(getStableComponentName(component), component); }));
        return components;
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAsMap(const AActor* actor, const std::string& tag)
    {
        return getComponentsByTagAnyAsMap<TComponent, TReturnAsComponent>(actor, {tag});
    }
    
    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAnyAsMap(const AActor* actor, const std::vector<std::string>& tags)
    {
        auto components = Std::toMap<std::string, TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor) |
            std::views::transform([&tags](auto component) { return std::make_pair(component, getComponentHasTags(component, tags)); }) |
            std::views::filter([](const auto& pair)       { const auto& [component, has_tags] = pair; return Std::any(has_tags); }) |
            std::views::transform([](const auto& pair)    { const auto& [component, has_tags] = pair; return std::make_pair(getStableComponentName(component), component); }));
        return components;
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAllAsMap(const AActor* actor, const std::vector<std::string>& tags)
    {
        auto components = Std::toMap<std::string, TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor) |
            std::views::transform([&tags](auto component) { return std::make_pair(component, getComponentHasTags(component, tags)); }) |
            std::views::filter([](const auto& pair)       { const auto& [component, has_tags] = pair; return Std::all(has_tags); }) |
            std::views::transform([](const auto& pair)    { const auto& [component, has_tags] = pair; return std::make_pair(getStableComponentName(component), component); }));
        return components;
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTypeAsMap(const AActor* actor)
    {
        auto components = Std::toMap<std::string, TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor) |
            std::views::transform([](auto component) { return std::make_pair(getStableComponentName(component), component); }));
        return components;
    }

    //
    // Get component by name or tag or type and return a TComponent* pointer
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByName(const AActor* actor, const std::string& name, bool assert_if_not_found = true)
    {
        bool assert_if_multiple_found = true;
        bool return_null_if_not_found = false;
        return getItem(getComponentsByName<TComponent, TReturnAsComponent>(
            actor, {name}, return_null_if_not_found), reinterpret_cast<TReturnAsComponent*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTag(const AActor* actor, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getComponentsByTag<TComponent, TReturnAsComponent>(actor, tag), reinterpret_cast<TReturnAsComponent*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTagAny(const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getComponentsByTagAny<TComponent, TReturnAsComponent>(actor, tags), reinterpret_cast<TReturnAsComponent*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTagAll(const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getComponentsByTagAll<TComponent, TReturnAsComponent>(actor, tags), reinterpret_cast<TReturnAsComponent*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByType(const AActor* actor, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getComponentsByType<TComponent, TReturnAsComponent>(actor), reinterpret_cast<TReturnAsComponent*>(nullptr), assert_if_not_found, assert_if_multiple_found);
    }

    //
    // Find struct by name
    //

    static UStruct* findStructByName(const UWorld* world, const std::string& name);

    //
    // Get and set object properties, uobject can't be const because we cast it to void*
    //

    static std::string getObjectPropertiesAsString(UObject* uobject);
    static std::string getObjectPropertiesAsString(void* value_ptr, const UStruct* ustruct);

    static void setObjectPropertiesFromString(UObject* uobject, const std::string& string);
    static void setObjectPropertiesFromString(void* value_ptr, const UStruct* ustruct, const std::string& string);

    //
    // Find property by name, get and set property values
    //

    struct PropertyDesc
    {
        FProperty* property_ = nullptr;
        void* value_ptr_ = nullptr;
    };

    static PropertyDesc findPropertyByName(UObject* uobject, const std::string& name);
    static PropertyDesc findPropertyByName(void* value_ptr, const UStruct* ustruct, const std::string& name);

    static std::string getPropertyValueAsString(const PropertyDesc& property_desc);
    static void setPropertyValueFromString(const PropertyDesc& property_desc, const std::string& string);

    //
    // Find function by name, call function, ufunction can't be const because we pass it to uobject->ProcessEvent(...), which expects non-const
    //

    static UFunction* findFunctionByName(const UClass* uclass, const std::string& name, EIncludeSuperFlag::Type include_super_flag = EIncludeSuperFlag::IncludeSuper);
    static std::map<std::string, std::string> callFunction(UObject* uobject, UFunction* ufunction); // useful for calling functions with no args
    static std::map<std::string, std::string> callFunction(UObject* uobject, UFunction* ufunction, const std::map<std::string, std::string>& args);

private:

    //
    // Helper functions for finding actors and getting components
    //

    static bool getActorHasStableName(const AActor* actor);
    static std::vector<bool> getActorHasTags(const AActor* actor, const std::vector<std::string>& tags);
    static std::vector<bool> getComponentHasTags(const UActorComponent* component, const std::vector<std::string>& tags);

    template <typename TValue>
    static const TValue& getItem(const std::vector<TValue>& vector, const TValue& default_val, bool assert_if_size_is_zero, bool assert_if_size_is_greater_than_one)
    {
        if (assert_if_size_is_zero) {
            SP_ASSERT(vector.size() != 0);
        }
        if (assert_if_size_is_greater_than_one) {
            SP_ASSERT(vector.size() <= 1);
        }
        if (vector.size() == 0) {
            return default_val;
        } else {
            return vector.at(0);
        }
    }

    //
    // Helper function for formatting array properties as strings in the same style as Unreal
    //

    static std::string getArrayPropertyValueAsFormattedString(const FProperty* inner_property, const std::vector<std::string>& inner_strings);
};
