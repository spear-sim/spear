//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>    // std::derived_from
#include <cstring>     // std::memcpy
#include <map>
#include <ranges>      // std::views::filter, std::views::transform
#include <string>
#include <type_traits> // std::is_base_of
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>        // TArray
#include <Containers/StringConv.h>   // TCHAR_TO_UTF8, UTF8_TO_TCHAR
#include <Containers/UnrealString.h> // FString::operator*
#include <EngineUtils.h>             // TActorIterator
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // TCHAR, TEXT
#include <UObject/NameTypes.h>       // FName

#include "SpCore/Assert.h"
#include "SpCore/StableNameComponent.h"
#include "SpCore/Std.h"

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

    static std::string toStdString(const FString& str)
    {
        // Note that the * operator for FString returns a pointer to the underlying string
        return std::string(TCHAR_TO_UTF8(*str));
    }

    static std::string toStdString(const FName& str)
    {
        // Note that str.ToString() converts FName to FString
        return toStdString(str.ToString());
    }

    static std::string toStdString(const TCHAR* str)
    {
        return std::string(TCHAR_TO_UTF8(str));
    }

    static FString toFString(const std::string& str)
    {
        return FString(UTF8_TO_TCHAR(str.c_str()));
    }

    static FName toFName(const std::string& str)
    {
        return FName(str.c_str());
    }

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
    // Find actors unconditionally and return an std::vector
    //

    static std::vector<AActor*> findActors(const UWorld* world)
    {
        return findActorsByType(world);
    }

    //
    // Find actors unconditionally and return an std::map
    //

    static std::map<std::string, AActor*> findActorsAsMap(const UWorld* world)
    {
        return findActorsByTypeAsMap(world);
    }

    //
    // Find actor by name or tag or type and return a TActor* pointer
    //

    template <CActor TActor = AActor>
    static TActor* findActorByName(const UWorld* world, const std::string& name, bool assert_if_not_found = true)
    {
        TActor* default_val                     = nullptr;
        bool return_null_if_not_found           = true;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = true;
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
    // Find actors by name or tag or type and return an std::vector
    //

    template <CActor TActor = AActor>
    static std::vector<TActor*> findActorsByName(const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        // This function is different because we need to return TActor* pointers in a particular order.
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
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::any(pair.second); }) |
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
        // This function is different because we need need to interact with TActorIterator directly.
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
            std::views::filter([&names](auto actor) { return Std::contains(names, toStdString(actor->GetName())); }) |
            std::views::transform([](auto actor)    { return std::make_pair(toStdString(actor->GetName()), actor); }));

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
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::any(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return std::make_pair(toStdString(actor->GetName()), actor); }));

        return actors;
    }

    template <CActor TActor = AActor>
    static std::map<std::string, TActor*> findActorsByTagAllAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        auto actors = Std::toMap<std::string, TActor*>(
            findActorsByType<TActor>(world) |
            std::views::transform([&tags](auto actor)  { return std::make_pair(actor, getActorHasTags(actor, tags)); }) |
            std::views::filter([](const auto& pair)    { const auto& [actor, has_tags] = pair; return Std::all(has_tags); }) |
            std::views::transform([](const auto& pair) { const auto& [actor, has_tags] = pair; return std::make_pair(toStdString(actor->GetName()), actor); }));

        return actors;
    }
    
    template <CActor TActor = AActor>
    static std::map<std::string, TActor*> findActorsByTypeAsMap(const UWorld* world)
    {
        auto actors = Std::toMap<std::string, TActor*>(
            findActorsByType<TActor>(world) |
            std::views::transform([](auto actor) { return std::make_pair(toStdString(actor->GetName()), actor); }));

        return actors;
    }

    //
    // Helper functions for finding actors
    //

    static std::vector<bool> getActorHasTags(const AActor* actor, const std::vector<std::string>& tags)
    {
        return Std::toVector<bool>(tags | std::views::transform([actor](const auto& tag) { return actor->ActorHasTag(toFName(tag)); }));
    }

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
        } else {
            return vec.at(0);
        }
    }

    //
    // Helper functions to get components.
    //

    template <CActorComponent TActorComponent>
    static std::vector<TActorComponent*> getComponentsByType(const AActor* actor)
    {
        TArray<TActorComponent*> components_tarray;
        actor->GetComponents<TActorComponent>(components_tarray);

        std::vector<TActorComponent*> components;
        for (auto component : components_tarray) {
            components.push_back(component);
        }

        return components;
    }

    //
    // Helper functions to get actor and component names.
    //

    static std::string getStableActorName(const AActor* actor)
    {
        SP_ASSERT(actor);

        std::vector<UStableNameComponent*> stable_name_components = Unreal::getComponentsByType<UStableNameComponent>(actor);
        SP_ASSERT(stable_name_components.size() == 1);

        UStableNameComponent* stable_name_component = stable_name_components.at(0);
        SP_ASSERT(stable_name_component);
        return toStdString(stable_name_component->StableName);
    }

    static void requestUpdateStableActorName(const AActor* actor)
    {
        SP_ASSERT(actor);

        std::vector<UStableNameComponent*> stable_name_components = Unreal::getComponentsByType<UStableNameComponent>(actor);
        SP_ASSERT(stable_name_components.size() == 1);

        UStableNameComponent* stable_name_component = stable_name_components.at(0);
        SP_ASSERT(stable_name_component);
        stable_name_component->update();
    }

    static std::string getStableComponentName(const USceneComponent* scene_component, bool include_actor_name = false)
    {
        SP_ASSERT(scene_component);

        std::string component_name = toStdString(scene_component->GetName());
        TArray<USceneComponent*> parents;
        scene_component->GetParentComponents(parents);
        for (auto parent : parents) {
            component_name = toStdString(parent->GetName()) + "." + component_name;
        }

        if (include_actor_name) {
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
};
