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

template <typename TActorComponent>
concept CActorComponent = std::derived_from<TActorComponent, UActorComponent>;

template <typename TSceneComponent>
concept CSceneComponent = CActorComponent<TSceneComponent> && std::derived_from<TSceneComponent, USceneComponent>;

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

    template <typename TData>
    static std::vector<TData> toStdVector(const TArray<TData>& src)
    {
        std::vector<TData> dest;
        for (auto& data : src) {
            dest.push_back(data);
        }
        return dest;
    }

    //
    // Helper functions to get actor and component names.
    //

    static std::string getStableActorName(const AActor* actor);
    static void setStableActorName(const AActor* actor, std::string stable_name);

    #if WITH_EDITOR // defined in an auto-generated header
        static void requestUpdateStableActorName(const AActor* actor);
    #endif

    static std::string getStableComponentName(const CActorComponent auto* actor_component, bool include_stable_actor_name = false)
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

    template <CActorComponent TActorComponent>
    static TActorComponent* createComponentInsideOwnerConstructor(AActor* owner, const std::string& name)
    {
        SP_ASSERT(owner);
        TActorComponent* actor_component = owner->CreateDefaultSubobject<TActorComponent>(toFName(name));
        SP_ASSERT(actor_component);
        return actor_component;
    }

    template <CSceneComponent TSceneComponent>
    static TSceneComponent* createComponentInsideOwnerConstructor(AActor* owner, const std::string& name)
    {
        SP_ASSERT(owner);
        TSceneComponent* scene_component = owner->CreateDefaultSubobject<TSceneComponent>(toFName(name));
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        return scene_component;
    }

    template <CSceneComponent TSceneComponent>
    static TSceneComponent* createComponentInsideOwnerConstructor(UObject* owner, USceneComponent* parent, const std::string& name)
    {
        SP_ASSERT(owner);
        TSceneComponent* scene_component = owner->CreateDefaultSubobject<TSceneComponent>(toFName(name));
        SP_ASSERT(scene_component);
        scene_component->SetupAttachment(parent);
        return scene_component;
    }

    template <CSceneComponent TSceneComponent>
    static TSceneComponent* createComponentInsideOwnerConstructor(USceneComponent* owner, const std::string& name)
    {
        return createComponentInsideOwnerConstructor<TSceneComponent>(owner, owner, name);
    }

    template <CActorComponent TActorComponent>
    static TActorComponent* createComponentOutsideOwnerConstructor(AActor* owner, const std::string& name)
    {
        SP_ASSERT(owner);
        TActorComponent* actor_component = NewObject<TActorComponent>(owner, toFName(name));
        SP_ASSERT(actor_component);
        actor_component->RegisterComponent();
        return actor_component;
    }

    template <CSceneComponent TSceneComponent>
    static TSceneComponent* createComponentOutsideOwnerConstructor(AActor* owner, const std::string& name)
    {
        SP_ASSERT(owner);
        TSceneComponent* scene_component = NewObject<TSceneComponent>(owner, toFName(name));
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        scene_component->RegisterComponent();
        return scene_component;
    }

    template <CSceneComponent TSceneComponent>
    static TSceneComponent* createComponentOutsideOwnerConstructor(UObject* owner, USceneComponent* parent, const std::string& name)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        TSceneComponent* scene_component = NewObject<TSceneComponent>(owner, toFName(name));
        SP_ASSERT(scene_component);
        scene_component->SetupAttachment(parent);
        scene_component->RegisterComponent();
        return scene_component;
    }

    template <CSceneComponent TSceneComponent>
    static TSceneComponent* createComponentOutsideOwnerConstructor(USceneComponent* owner, const std::string& name)
    {
        return createComponentOutsideOwnerConstructor<TSceneComponent>(owner, owner, name);
    }

    // 
    // Find actors unconditionally and return an std::vector or an std::map
    //

    static std::vector<AActor*> findActors(const UWorld* world);
    static std::map<std::string, AActor*> findActorsAsMap(const UWorld* world);

    //
    // Find actors by name or tag or type and return an std::vector
    //

    template <CActor TActor = AActor>
    static std::vector<TActor*> findActorsByName(const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        std::map<std::string, TActor*> actor_map = findActorsByNameAsMap<TActor>(world, names);
        std::vector<TActor*> actors;
        if (return_null_if_not_found) {
            actors = Std::toVector<TActor*>(
                names |
                std::views::transform([&actor_map](const auto& name) { return Std::containsKey(actor_map, name) ? actor_map.at(name) : nullptr; }));
        } else {
            actors = Std::toVector<TActor*>(
                names |
                std::views::filter([&actor_map](const auto& name)    { return Std::containsKey(actor_map, name); }) |
                std::views::transform([&actor_map](const auto& name) { return actor_map.at(name); }));
        }
        return actors;
    }

    template <CActor TActor = AActor>
    static std::vector<TActor*> findActorsByTag(const UWorld* world, const std::string& tag)
    {
        return findActorsByTagAny<TActor>(world, {tag});
    }

    template <CActor TActor = AActor>
    static std::vector<TActor*> findActorsByTagAny(const UWorld* world, const std::vector<std::string>& tags)
    {
        auto actors = Std::toVector<TActor*>(
            findActorsByType<TActor>(world) |
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::any(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return actor; }));
        return actors;
    }

    template <CActor TActor = AActor>
    static std::vector<TActor*> findActorsByTagAll(const UWorld* world, const std::vector<std::string>& tags)
    {
        auto actors = Std::toVector<TActor*>(
            findActorsByType<TActor>(world) |
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::all(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return actor; }));
        return actors;
    }

    template <CActor TActor = AActor>
    static std::vector<TActor*> findActorsByType(const UWorld* world)
    {
        SP_ASSERT(world);
        std::vector<TActor*> actors;
        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            TActor* actor = *itr;
            SP_ASSERT(actor);
            actors.push_back(actor);
        }
        return actors;
    }

    //
    // Find actors by name or tag or type and return an std::map
    //

    template <CActor TActor = AActor>
    static std::map<std::string, TActor*> findActorsByNameAsMap(const UWorld* world, const std::vector<std::string>& names)
    {
        auto actors = Std::toMap<std::string, TActor*>(
            findActorsByType<TActor>(world) |
            std::views::filter([](auto actor)       { return getActorHasStableName(actor); }) |
            std::views::filter([&names](auto actor) { return Std::contains(names, getStableActorName(actor)); }) |
            std::views::transform([](auto actor)    { return std::make_pair(getStableActorName(actor), actor); }));
        return actors;
    }

    template <CActor TActor = AActor>
    static std::map<std::string, TActor*> findActorsByTagAsMap(const UWorld* world, const std::string& tag)
    {
        return findActorsByTagAnyAsMap(world, {tag});
    }
    
    template <CActor TActor = AActor>
    static std::map<std::string, TActor*> findActorsByTagAnyAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        auto actors = Std::toMap<std::string, TActor*>(
            findActorsByType<TActor>(world) |
            std::views::filter([](auto actor)          { return getActorHasStableName(actor); }) |
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::any(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return std::make_pair(getStableActorName(actor), actor); }));
        return actors;
    }

    template <CActor TActor = AActor>
    static std::map<std::string, TActor*> findActorsByTagAllAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        auto actors = Std::toMap<std::string, TActor*>(
            findActorsByType<TActor>(world) |
            std::views::filter([](auto actor)          { return getActorHasStableName(actor); }) |
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::all(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return std::make_pair(getStableActorName(actor), actor); }));
        return actors;
    }
    
    template <CActor TActor = AActor>
    static std::map<std::string, TActor*> findActorsByTypeAsMap(const UWorld* world)
    {
        auto actors = Std::toMap<std::string, TActor*>(
            findActorsByType<TActor>(world) |
            std::views::filter([](auto actor)    { return getActorHasStableName(actor); }) |
            std::views::transform([](auto actor) { return std::make_pair(getStableActorName(actor), actor); }));
        return actors;
    }

    //
    // Find actor by name or tag or type and return a TActor* pointer
    //

    template <CActor TActor = AActor>
    static TActor* findActorByName(const UWorld* world, const std::string& name, bool assert_if_not_found = true)
    {
        TActor* default_val                     = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = true;
        bool return_null_if_not_found           = false;
        return getItem(findActorsByName<TActor>(world, {name}, return_null_if_not_found), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <CActor TActor = AActor>
    static TActor* findActorByTag(const UWorld* world, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActor* default_val                     = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(findActorsByTag<TActor>(world, tag), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <CActor TActor = AActor>
    static TActor* findActorByTagAny(const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActor* default_val                     = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(findActorsByTagAny<TActor>(world, tags), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <CActor TActor = AActor>
    static TActor* findActorByTagAll(const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActor* default_val                     = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(findActorsByTagAll<TActor>(world, tags), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <CActor TActor = AActor>
    static TActor* findActorByType(const UWorld* world, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActor* default_val                     = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(findActorsByType<TActor>(world), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    //
    // Get components unconditionally and return an std::vector or an std::map
    //

    std::vector<UActorComponent*> getComponents(const AActor* actor);
    std::map<std::string, UActorComponent*> getComponentsAsMap(const AActor* actor);

    //
    // Get components by name or tag or type and return an std::vector
    //

    template <CActorComponent TActorComponent = UActorComponent>
    static std::vector<TActorComponent*> getComponentsByName(const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        std::map<std::string, TActorComponent*> component_map = getComponentsByNameAsMap<TActorComponent>(actor, names);
        std::vector<TActorComponent*> components;
        if (return_null_if_not_found) {
            components = Std::toVector<TActorComponent*>(
                names |
                std::views::transform([&component_map](const auto& name) { return Std::containsKey(component_map, name) ? component_map.at(name) : nullptr; }));
        } else {
            components = Std::toVector<TActorComponent*>(
                names |
                std::views::filter([&component_map](const auto& name)    { return Std::containsKey(component_map, name); }) |
                std::views::transform([&component_map](const auto& name) { return component_map.at(name); }));
        }
        return components;
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static std::vector<TActorComponent*> getComponentsByTag(const AActor* actor, const std::string& tag)
    {
        return getComponentsByTagAny<TActorComponent>(actor, {tag});
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static std::vector<TActorComponent*> getComponentsByTagAny(const AActor* actor, const std::vector<std::string>& tags)
    {
        auto components = Std::toVector<TActorComponent*>(
            getComponentsByType<TActorComponent>(actor) |
            std::views::transform([&tags](auto component) { return std::make_pair(component, getComponentHasTags(component, tags)); }) |
            std::views::filter([](const auto& pair)       { const auto& [component, has_tags] = pair; return Std::any(has_tags); }) |
            std::views::transform([](const auto& pair)    { const auto& [component, has_tags] = pair; return component; }));
        return components;
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static std::vector<TActorComponent*> getComponentsByTagAll(const AActor* actor, const std::vector<std::string>& tags)
    {
        auto components = Std::toVector<TActorComponent*>(
            getComponentsByType<TActorComponent>(actor) |
            std::views::transform([&tags](auto component) { return std::make_pair(component, getComponentHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)       { const auto& [component, has_tags] = pair; return Std::all(has_tags); }) |
            std::views::transform([](const auto& pair)    { const auto& [component, has_tags] = pair; return component; }));
        return components;
    }


    template <CActorComponent TActorComponent = UActorComponent>
    static std::vector<TActorComponent*> getComponentsByType(const AActor* actor)
    {
        TArray<TActorComponent*> components;
        actor->GetComponents<TActorComponent>(components);
        return toStdVector(components);
    }

    //
    // Get components by name or tag or type and return an std::map
    //

    template <CActorComponent TActorComponent = UActorComponent>
    static std::map<std::string, TActorComponent*> getComponentsByNameAsMap(const AActor* actor, const std::vector<std::string>& names)
    {
        auto components = Std::toMap<std::string, TActorComponent*>(
            getComponentsByType<TActorComponent>(actor) |
            std::views::filter([&names](auto component) { return Std::contains(names, getStableComponentName(component)); }) |
            std::views::transform([](auto component)    { return std::make_pair(getStableComponentName(component), component); }));
        return components;
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static std::map<std::string, TActorComponent*> getComponentsByTagAsMap(const AActor* actor, const std::string& tag)
    {
        return getComponentsByTagAnyAsMap(actor, {tag});
    }
    
    template <CActorComponent TActorComponent = UActorComponent>
    static std::map<std::string, TActorComponent*> getComponentsByTagAnyAsMap(const AActor* actor, const std::vector<std::string>& tags)
    {
        auto components = Std::toMap<std::string, TActorComponent*>(
            getComponentsByType<TActorComponent>(actor) |
            std::views::transform([&tags](auto component) { return std::make_pair(component, getComponentHasTags(component, tags)); }) |
            std::views::filter([](const auto& pair)       { const auto& [component, has_tags] = pair; return Std::any(has_tags); }) |
            std::views::transform([](const auto& pair)    { const auto& [component, has_tags] = pair; return std::make_pair(getStableComponentName(component), component); }));
        return components;
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static std::map<std::string, TActorComponent*> getComponentsByTagAllAsMap(const AActor* actor, const std::vector<std::string>& tags)
    {
        auto components = Std::toMap<std::string, TActorComponent*>(
            getComponentsByType<TActorComponent>(actor) |
            std::views::transform([&tags](auto component) { return std::make_pair(component, getComponentHasTags(component, tags)); }) |
            std::views::filter([](const auto& pair)       { const auto& [component, has_tags] = pair; return Std::all(has_tags); }) |
            std::views::transform([](const auto& pair)    { const auto& [component, has_tags] = pair; return std::make_pair(getStableComponentName(component), component); }));
        return components;
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static std::map<std::string, TActorComponent*> getComponentsByTypeAsMap(const AActor* actor)
    {
        auto components = Std::toMap<std::string, TActorComponent*>(
            getComponentsByType<TActorComponent>(actor) |
            std::views::transform([](auto component) { return std::make_pair(getStableComponentName(component), component); }));
        return components;
    }

    //
    // Get component by name or tag or type and return a TActorComponent* pointer.
    //

    template <CActorComponent TActorComponent = UActorComponent>
    static TActorComponent* getComponentByName(const AActor* actor, const std::string& name, bool assert_if_not_found = true)
    {
        TActorComponent* default_val            = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = true;
        bool return_null_if_not_found           = false;
        return getItem(getComponentsByName<TActorComponent>(actor, {name}, return_null_if_not_found), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static TActorComponent* getComponentByTag(const AActor* actor, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActorComponent* default_val            = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(getComponentsByTag<TActorComponent>(actor, tag), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static TActorComponent* getComponentByTagAny(const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActorComponent* default_val            = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(getComponentsByTagAny<TActorComponent>(actor, tags), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static TActorComponent* getComponentByTagAll(const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActorComponent* default_val            = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(getComponentsByTagAll<TActorComponent>(actor, tags), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <CActorComponent TActorComponent = UActorComponent>
    static TActorComponent* getComponentByType(const AActor* actor, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActorComponent* default_val            = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(getComponentsByType<TActorComponent>(actor), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    //
    // Helper functions for finding actors and getting components
    //

    static bool getActorHasStableName(const AActor* actor);
    static std::vector<bool> getActorHasTags(const AActor* actor, const std::vector<std::string>& tags);
    static std::vector<bool> getComponentHasTags(const UActorComponent* component, const std::vector<std::string>& tags);

    template <typename TData>
    static const TData& getItem(const std::vector<TData>& vec, const TData& default_val, bool assert_if_size_is_zero, bool assert_if_size_is_greater_than_one)
    {
        if (assert_if_size_is_zero) {
            SP_ASSERT(vec.size() != 0);
        }
        if (assert_if_size_is_greater_than_one) {
            SP_ASSERT(vec.size() <= 1);
        }
        if (vec.size() == 0) {
            return default_val;
        }
        else {
            return vec.at(0);
        }
    }

    //
    // Find struct by name
    //

    static UStruct* findStructByName(const UWorld* world, const std::string& name);

    //
    // Get and set object properties
    //

    static std::string getObjectPropertiesAsString(UObject* uobject);
    static std::string getObjectPropertiesAsString(void* value_ptr, const UStruct* ustruct);

    static void setObjectPropertiesFromString(UObject* uobject, const std::string& string);
    static void setObjectPropertiesFromString(void* value_ptr, UStruct* ustruct, const std::string& string);

    //
    // Find property by name, get and set property values
    //

    struct PropertyDesc
    {
        void* value_ptr_ = nullptr;
        FProperty* property_ = nullptr;
    };

    static PropertyDesc findPropertyByName(UObject* uobject, const std::string& name);
    static PropertyDesc findPropertyByName(void* value_ptr, const UStruct* ustruct, const std::string& name);

    static std::string getPropertyValueAsString(const PropertyDesc& property_desc);
    static void setPropertyValueFromString(const PropertyDesc& property_desc, const std::string& string);

    //
    // Find function by name, call function
    //

    static UFunction* findFunctionByName(UClass* uclass, const std::string& name, EIncludeSuperFlag::Type include_super_flag = EIncludeSuperFlag::IncludeSuper);
    static std::map<std::string, std::string> callFunction(UObject* uobject, UFunction* ufunction, const std::map<std::string, std::string>& args);
};
