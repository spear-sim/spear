//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int32_t, int64_t

#include <concepts>    // std::derived_from
#include <map>
#include <ranges>      // std::views::filter, std::views::transform
#include <string>
#include <type_traits> // std::remove_pointer_t, std::underlying_type_t
#include <utility>     // std::make_pair
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>
#include <Containers/UnrealString.h>        // FString::operator*
#include <Engine/World.h>
#include <EngineUtils.h>                    // TActorIterator
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>                   // int32, int64, TCHAR
#include <Subsystems/Subsystem.h>
#include <Templates/Casts.h>
#include <UObject/Class.h>                  // EIncludeSuperFlag, UClass, UEnum, UStruct
#include <UObject/NameTypes.h>              // FName
#include <UObject/Object.h>                 // UObject
#include <UObject/ReflectedTypeAccessors.h> // StaticEnum
#include <UObject/UnrealType.h>             // FProperty
#include <UObject/UObjectGlobals.h>         // NewObject

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

//
// Concepts for Unreal objects
//

template <typename TObject>
concept CObject =
    std::derived_from<TObject, UObject>;

// In the CStruct and CClass concepts below, there does not seem to be a clean way to encode the desired
// std::derived_from<...> relationships directly in the requires(...) { ... } statement of each concept. This
// is because, e.g., the type TStruct is implicitly passed as the first template parameter to std::derived_from<...>,
// but we actually need the type std::remove_pointer_t<TStruct> to be passed instead, in order for std::derived_from<...>
// to behave as expected. So we encode the std::derived_from<...> constraint outside the requires(...) { ... }
// statement in each concept, where we can manipulate TStruct and TClass more freely.

template <typename TEnum>
concept CEnum =
    requires() {
        { StaticEnum<TEnum>() } -> std::same_as<UEnum*>;
    };

template <typename TStruct>
concept CStruct =
    requires() {
        { TStruct::StaticStruct() }; // can't use std::same_as<UStruct*> because the type of the returned pointer might be derived from UStruct
    } &&
    std::derived_from<std::remove_pointer_t<decltype(TStruct::StaticStruct())>, UStruct>;

template <typename TClass>
concept CClass =
    CObject<TClass> &&
    requires() {
        { TClass::StaticClass() }; // can't use std::same_as<UClass*> because the type of the returned pointer might be derived from UClass
    } &&
    std::derived_from<std::remove_pointer_t<decltype(TClass::StaticClass())>, UClass>;

template <typename TComponent>
concept CComponent =
    CObject<TComponent> &&
    std::derived_from<TComponent, UActorComponent>;

template <typename TSceneComponent>
concept CSceneComponent =
    CComponent<TSceneComponent> &&
    std::derived_from<TSceneComponent, USceneComponent>;

template <typename TActor>
concept CActor =
    CObject<TActor> &&
    std::derived_from<TActor, AActor>;

template <typename TStableNameObject>
concept CStableNameObject =
    CActor<TStableNameObject> || CComponent<TStableNameObject>;

template <typename TParent>
concept CParent =
    CActor<TParent> || CSceneComponent<TParent>;

template <typename TSubsystem>
concept CSubsystem =
    CClass<TSubsystem> &&
    std::derived_from<TSubsystem, USubsystem>;

template <typename TSubsystemProvider>
concept CSubsystemProvider =
    CClass<TSubsystemProvider> &&
    requires(TSubsystemProvider subsystem_provider) {
        { subsystem_provider.template GetSubsystem<USubsystem>() } -> std::same_as<USubsystem*>;
    };

template <typename TSubsystemBaseProvider>
concept CSubsystemBaseProvider =
    CSubsystemProvider<TSubsystemBaseProvider> &&
    requires(TSubsystemBaseProvider subsystem_base_provider, UClass* uclass) {
        { subsystem_base_provider.GetSubsystemBase(nullptr) };
    } &&
    std::derived_from<std::remove_pointer_t<decltype(TSubsystemBaseProvider().GetSubsystemBase(nullptr))>, USubsystem>;

//
// General-purpose functions for working with Unreal objects.
//

class SPCORE_API Unreal
{
public:
    Unreal() = delete;
    ~Unreal() = delete;

    //
    // Helper function for structs
    //

    template <CClass TClass>
    static UStruct* getStaticStruct()
    {
        return TClass::StaticClass();
    }

    template <CStruct TStruct>
    static UStruct* getStaticStruct()
    {
        return TStruct::StaticStruct();
    }

    static std::string getStaticStructName(const UStruct* ustruct);

    //
    // Find special struct by name. For this function to behave as expected, ASpSpecialStructActor must have
    // a UPROPERTY defined on it named TypeName_ of type TypeName.
    //

    static UStruct* findSpecialStructByName(const std::string& struct_name);

    //
    // Get and set object properties, uobject can't be const because we cast it to void*
    //

    static std::string getObjectPropertiesAsString(const UObject* uobject);
    static std::string getObjectPropertiesAsString(const void* value_ptr, const UStruct* ustruct);

    static void setObjectPropertiesFromString(UObject* uobject, const std::string& string);
    static void setObjectPropertiesFromString(void* value_ptr, const UStruct* ustruct, const std::string& string);

    //
    // Find property by name, get and set property values, uobject can't be const because we cast it to
    // (non-const) void*, value_ptr can't be const because we assign to PropertyDesc::value_ptr_.
    //

    struct PropertyDesc
    {
        FProperty* property_ = nullptr;
        void* value_ptr_ = nullptr;
    };

    static PropertyDesc findPropertyByName(UObject* uobject, const std::string& property_name);
    static PropertyDesc findPropertyByName(void* value_ptr, const UStruct* ustruct, const std::string& property_name);

    static std::string getPropertyValueAsString(const PropertyDesc& property_desc);
    static void setPropertyValueFromString(const PropertyDesc& property_desc, const std::string& string);
    static void setPropertyValueFromJsonValue(const PropertyDesc& property_desc, TSharedPtr<FJsonValue> json_value);

    //
    // Find function by name, call function, uobject can't be const because we call uobject->ProcessEvent(...)
    // which is non-const, ufunction can't be const because we call because we pass it to uobject->ProcessEvent(...)
    // which expects non-const
    //

    static UFunction* findFunctionByName(const UClass* uclass, const std::string& function_name, EIncludeSuperFlag::Type include_super_flag = EIncludeSuperFlag::IncludeSuper);
    static std::map<std::string, std::string> callFunction(const UWorld* world, UObject* uobject, UFunction* ufunction, const std::map<std::string, std::string>& args = {}, const std::string& world_context = "WorldContextObject");

    //
    // Helper functions to create components.
    //
    // If we are inside the constructor of an owner AActor (either directly or indirectly via the constructor
    // of a child component), then we must call CreateDefaultSubobject.
    // 
    // If we are outside the constructor of an owner AActor (either directly or indirectly via the
    // constructor of a child component), then we must call NewObject and RegisterComponent.
    // 
    // If we are creating a USceneComponent that is intended to be a root component, then we must call
    // SetRootComponent to attach the newly created component to its parent AActor.
    //
    // If we are creating a USceneComponent that is not intended to be a root component, then its parent must
    // also be a USceneComponent, and we must call SetupAttachment to attach the newly created component to
    // its parent component.
    //
    // SetupAttachment must be called before RegisterComponent.
    //

    template <CComponent TComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* createComponentInsideOwnerConstructor(AActor* owner, const std::string& component_name)
    {
        SP_ASSERT(owner);
        TReturnAsComponent* actor_component = owner->CreateDefaultSubobject<TComponent>(toFName(component_name));
        SP_ASSERT(actor_component);
        return actor_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentInsideOwnerConstructor(AActor* owner, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        TReturnAsSceneComponent* scene_component = owner->CreateDefaultSubobject<TSceneComponent>(toFName(scene_component_name));
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        return scene_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentInsideOwnerConstructor(UObject* owner, USceneComponent* parent, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        TReturnAsSceneComponent* scene_component = owner->CreateDefaultSubobject<TSceneComponent>(toFName(scene_component_name));
        SP_ASSERT(scene_component);
        scene_component->SetupAttachment(parent);
        return scene_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentInsideOwnerConstructor(USceneComponent* owner, const std::string& scene_component_name)
    {
        return createComponentInsideOwnerConstructor<TSceneComponent, TReturnAsSceneComponent>(owner, owner, scene_component_name);
    }

    template <CComponent TComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* createComponentOutsideOwnerConstructor(AActor* owner, const std::string& component_name)
    {
        SP_ASSERT(owner);
        TReturnAsComponent* actor_component = NewObject<TComponent>(owner, toFName(component_name));
        SP_ASSERT(actor_component);
        actor_component->RegisterComponent();
        return actor_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentOutsideOwnerConstructor(AActor* owner, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        TReturnAsSceneComponent* scene_component = NewObject<TSceneComponent>(owner, toFName(scene_component_name));
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        scene_component->RegisterComponent();
        return scene_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentOutsideOwnerConstructor(UObject* owner, USceneComponent* parent, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        TReturnAsSceneComponent* scene_component = NewObject<TSceneComponent>(owner, toFName(scene_component_name));
        SP_ASSERT(scene_component);
        scene_component->SetupAttachment(parent);
        scene_component->RegisterComponent();
        return scene_component;
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createComponentOutsideOwnerConstructor(USceneComponent* owner, const std::string& scene_component_name)
    {
        return createComponentOutsideOwnerConstructor<TSceneComponent, TReturnAsSceneComponent>(owner, owner, scene_component_name);
    }

    // 
    // Find actors unconditionally and return an std::vector or an std::map
    //

    static std::vector<AActor*> findActors(const UWorld* world);
    static std::map<std::string, AActor*> findActorsAsMap(const UWorld* world);

    //
    // Get components unconditionally and return an std::vector or an std::map
    //

    static std::vector<UActorComponent*> getComponents(const AActor* actor);
    static std::map<std::string, UActorComponent*> getComponentsAsMap(const AActor* actor);

    //
    // Get children components unconditionally and return an std::vector or an std::map
    //

    static std::vector<USceneComponent*> getChildrenComponents(const USceneComponent* parent, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsAsMap(const USceneComponent* parent, bool include_all_descendants = true);

    //
    // Find actor and component by path and return both
    //

    template <CActor TActor = AActor, CComponent TComponent = UActorComponent, CActor TReturnAsActor = TActor, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TActor, TReturnAsActor> && std::derived_from<TComponent, TReturnAsComponent>
    static std::pair<TReturnAsActor*, TReturnAsComponent*> findActorAndComponentByPath(const UWorld* world, const AActor* owner, const std::string& path)
    {
        std::pair<TReturnAsActor*, TReturnAsComponent*> pair = std::make_pair(nullptr, nullptr);

        std::vector<std::string> path_tokens = Std::tokenize(path, ":");
        SP_ASSERT(path_tokens.size() == 1 || path_tokens.size() == 2);

        if (path_tokens.size() == 1) {
            pair.first = const_cast<TReturnAsActor*>(Cast<TReturnAsActor>(owner));
            pair.second = Unreal::getComponentByName<TComponent, TReturnAsComponent>(pair.first, path_tokens.at(0));
        } else if (path_tokens.size() == 2) {
            pair.first = Unreal::findActorByName<TActor, TReturnAsActor>(world, path_tokens.at(0));
            pair.second = Unreal::getComponentByName<TComponent, TReturnAsComponent>(pair.first, path_tokens.at(1));
        }

        return pair;
    }

    //
    // Find actors by name or tag or type and return an std::vector
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByName(const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsActor*> actors;
        std::map<std::string, TReturnAsActor*> all_actors_map = toMap<TReturnAsActor>(findActorsByType<TActor>(world));

        if (return_null_if_not_found) {
            actors = Std::at(all_actors_map, actor_names, nullptr);
        } else {
            std::vector<std::string> actor_names_found_in_map =
                Std::toVector<std::string>(actor_names | std::views::filter([&all_actors_map](const auto& name) { return Std::containsKey(all_actors_map, name); }));
            actors = Std::at(all_actors_map, actor_names_found_in_map);
        }

        return actors;
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTag(const UWorld* world, const std::string& tag)
    {
        return findActorsByTagAny<TActor, TReturnAsActor>(world, {tag});
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTagAny(const UWorld* world, const std::vector<std::string>& tags)
    {
        return Std::toVector<TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::filter([&tags](auto actor) { return Std::any(Std::contains(getTags(actor), tags)); }));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTagAll(const UWorld* world, const std::vector<std::string>& tags)
    {
        return Std::toVector<TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::filter([&tags](auto actor) { return Std::all(Std::contains(getTags(actor), tags)); }));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByType(const UWorld* world)
    {
        return findActorsByClass<TReturnAsActor>(world, TActor::StaticClass());
    }

    template <CActor TReturnAsActor = AActor>
    static std::vector<TReturnAsActor*> findActorsByClass(const UWorld* world, UClass* uclass) // uclass can't be const because then we can't construct itr
    {
        SP_ASSERT(world);
        SP_ASSERT(uclass);
        std::vector<TReturnAsActor*> actors;
        for (TActorIterator<TReturnAsActor> itr(world, uclass); itr; ++itr) {
            TReturnAsActor* actor = *itr;
            SP_ASSERT(actor);
            actors.push_back(actor);
        }
        return actors;
    }

    //
    // Find actors by name or tag or type and return an std::map
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByNameAsMap(const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found = true)
    {
        std::map<std::string, TReturnAsActor*> actor_map;
        std::vector<std::string> unique_actor_names = Std::unique(actor_names);

        if (return_null_if_not_found) {
            actor_map = Std::zip(unique_actor_names, findActorsByName<TActor, TReturnAsActor>(world, unique_actor_names, return_null_if_not_found));
        } else {
            std::vector<TActor*> actors = findActorsByName<TActor>(world, unique_actor_names, return_null_if_not_found);
            actor_map = toMap<TReturnAsActor>(findActorsByName<TActor>(world, unique_actor_names, return_null_if_not_found));
        }

        return actor_map;
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAsMap(const UWorld* world, const std::string& tag)
    {
        return toMap<TReturnAsActor>(findActorsByTag<TActor>(world, tag));
    }
    
    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAnyAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        return toMap<TReturnAsActor>(findActorsByTagAny<TActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAllAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        return toMap<TReturnAsActor>(findActorsByTagAll<TActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTypeAsMap(const UWorld* world)
    {
        return toMap<TReturnAsActor>(findActorsByType<TActor>(world));
    }

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByClassAsMap(const UWorld* world, UClass* uclass)
    {
        return toMap<TReturnAsActor>(findActorsByClass<TReturnAsActor>(world, uclass));
    }

    //
    // Find actor by name or tag or type and return a pointer
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByName(const UWorld* world, const std::string& actor_name)
    {
        bool return_null_if_not_found = false;
        return getItem(findActorsByName<TActor, TReturnAsActor>(world, {actor_name}, return_null_if_not_found));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTag(const UWorld* world, const std::string& tag)
    {
        return getItem(findActorsByTag<TActor, TReturnAsActor>(world, tag));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTagAny(const UWorld* world, const std::vector<std::string>& tags)
    {
        return getItem(findActorsByTagAny<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTagAll(const UWorld* world, const std::vector<std::string>& tags)
    {
        return getItem(findActorsByTagAll<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByType(const UWorld* world)
    {
        return getItem(findActorsByType<TActor, TReturnAsActor>(world));
    }

    template <CActor TReturnAsActor = AActor>
    static TReturnAsActor* findActorByClass(const UWorld* world, UClass* uclass)
    {
        return getItem(findActorsByClass<TReturnAsActor>(world, uclass));
    }

    //
    // Get components by name or tag or type and return an std::vector
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByName(
        const AActor* actor, const std::vector<std::string>& component_names, bool include_from_child_actors = false, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsComponent*> components;
        std::map<std::string, TReturnAsComponent*> all_components_map = toMap<TReturnAsComponent>(getComponentsByType<TComponent>(actor, include_from_child_actors));

        if (return_null_if_not_found) {
            components = Std::at(all_components_map, component_names, nullptr);
        } else {
            std::vector<std::string> component_names_found_in_map =
                Std::toVector<std::string>(component_names | std::views::filter([&all_components_map](const auto& name) { return Std::containsKey(all_components_map, name); }));
            components = Std::at(all_components_map, component_names_found_in_map);
        }

        return components;
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTag(const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return getComponentsByTagAny<TComponent, TReturnAsComponent>(actor, {tag}, include_from_child_actors);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagAny(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return Std::toVector<TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor, include_from_child_actors) |
            std::views::filter([&tags](auto component) { return Std::any(Std::contains(getTags(component), tags)); }));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagAll(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return Std::toVector<TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor, include_from_child_actors) |
            std::views::filter([&tags](auto component) { return Std::all(Std::contains(getTags(component), tags)); }));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByType(const AActor* actor, bool include_from_child_actors = false)
    {
        return getComponentsByClass<TReturnAsComponent>(actor, TComponent::StaticClass(), include_from_child_actors);
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::vector<TReturnAsComponent*> getComponentsByClass(const AActor* actor, UClass* uclass, bool include_from_child_actors = false)
    {
        SP_ASSERT(actor);
        SP_ASSERT(uclass);
        TArray<UActorComponent*> components_tarray;
        actor->GetComponents(uclass, components_tarray, include_from_child_actors); // uclass can't be const because then we can't pass to GetComponents(...)
        std::vector<UActorComponent*> components = toStdVector(components_tarray);
        SP_ASSERT(!Std::contains(components, nullptr));
        std::vector<TReturnAsComponent*> return_as_components = Std::toVector<TReturnAsComponent*>(
            components |
            std::views::transform([](auto component) { return Cast<TReturnAsComponent>(component); })); // no RTTI available, so use Cast instead of dynamic_cast
        SP_ASSERT(!Std::contains(return_as_components, nullptr));
        return return_as_components;
    }

    //
    // Get components by name or tag or type and return an std::map
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByNameAsMap(
        const AActor* actor, const std::vector<std::string>& component_names, bool include_from_child_actors = false, bool return_null_if_not_found = true)
    {
        std::map<std::string, TReturnAsComponent*> component_map;
        std::vector<std::string> unique_component_names = Std::unique(component_names);

        if (return_null_if_not_found) {
            component_map = Std::zip(unique_component_names, getComponentsByName<TComponent, TReturnAsComponent>(actor, unique_component_names, return_null_if_not_found));
        } else {
            std::vector<TComponent*> components = getComponentsByName<TComponent>(actor, unique_component_names, return_null_if_not_found);
            component_map = toMap<TReturnAsComponent>(components);
        }

        return component_map;
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAsMap(const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return toMap<TReturnAsComponent>(getComponentsByTag<TComponent>(actor, tag, include_from_child_actors));
    }
    
    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAnyAsMap(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toMap<TReturnAsComponent>(getComponentsByTagAny<TComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAllAsMap(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toMap<TReturnAsComponent>(getComponentsByTagAll<TComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTypeAsMap(const AActor* actor, bool include_from_child_actors = false)
    {
        return toMap<TReturnAsComponent>(getComponentsByType<TComponent>(actor, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByClassAsMap(const AActor* actor, UClass* uclass, bool include_from_child_actors = false)
    {
        return toMap<TReturnAsComponent>(getComponentsByClass<TReturnAsComponent>(actor, uclass, include_from_child_actors));
    }

    //
    // Get component by name or tag or type and return a pointer
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByName(const AActor* actor, const std::string& component_name, bool include_from_child_actors = false)
    {
        bool return_null_if_not_found = false;
        return getItem(getComponentsByName<TComponent, TReturnAsComponent>(actor, {component_name}, include_from_child_actors, return_null_if_not_found));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTag(const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return getItem(getComponentsByTag<TComponent, TReturnAsComponent>(actor, tag, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTagAny(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return getItem(getComponentsByTagAny<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTagAll(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return getItem(getComponentsByTagAll<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByType(const AActor* actor, bool include_from_child_actors = false)
    {
        return getItem(getComponentsByType<TComponent, TReturnAsComponent>(actor, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static TReturnAsComponent* getComponentByClass(const AActor* actor, UClass* uclass, bool include_from_child_actors = false)
    {
        return getItem(getComponentsByClass<TReturnAsComponent>(actor, uclass, include_from_child_actors));
    }

    //
    // Get children components by name or tag or type and return an std::vector
    //

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByName(
        const TParent* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants = true, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsSceneComponent*> components;
        std::map<std::string, TReturnAsSceneComponent*> all_components_map = toMap<TReturnAsSceneComponent>(getChildrenComponentsByType<TSceneComponent>(parent, include_all_descendants));

        if (return_null_if_not_found) {
            components = Std::at(all_components_map, children_component_names, nullptr);
        } else {
            std::vector<std::string> scene_component_names_found_in_map =
                Std::toVector<std::string>(children_component_names | std::views::filter([&all_components_map](const auto& name) { return Std::containsKey(all_components_map, name); }));
            components = Std::at(all_components_map, scene_component_names_found_in_map);
        }

        return components;
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTag(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return getChildrenComponentsByTagAny<TParent, TSceneComponent, TReturnAsSceneComponent>(parent, {tag}, include_all_descendants);
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTagAny(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return Std::toVector<TReturnAsSceneComponent*>(
            getChildrenComponentsByType<TSceneComponent, TReturnAsSceneComponent>(parent, include_all_descendants) |
            std::views::filter([&tags](auto component) { return Std::any(Std::contains(getTags(component), tags)); }));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTagAll(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return Std::toVector<TReturnAsSceneComponent*>(
            getChildrenComponentsByType<TSceneComponent, TReturnAsSceneComponent>(parent, include_all_descendants) |
            std::views::filter([&tags](auto component) { return Std::all(Std::contains(getTags(component), tags)); }));
    }

    // specialization for AActor
    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByType(const AActor* parent, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        USceneComponent* root = parent->GetRootComponent();
        std::vector<TReturnAsSceneComponent*> components;
        if (Cast<TSceneComponent>(root)) { // no RTTI available, so use Cast instead of dynamic_cast
            components.push_back(static_cast<TReturnAsSceneComponent*>(root));
        }
        if (root && include_all_descendants) {
            std::vector<TReturnAsSceneComponent*> children = getChildrenComponentsByType<TSceneComponent, TReturnAsSceneComponent>(root, include_all_descendants);
            components.insert(components.end(), children.begin(), children.end()); // TODO: replace with components.append_range(children) in C++23
        }
        return components;
    }

    // specialization for USceneComponent
    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByType(const USceneComponent* parent, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        TArray<USceneComponent*> children_tarray;
        parent->GetChildrenComponents(include_all_descendants, children_tarray);
        std::vector<USceneComponent*> children = toStdVector(children_tarray);
        SP_ASSERT(!Std::contains(children, nullptr));
        return Std::toVector<TReturnAsSceneComponent*>(
            children |
            std::views::filter([](auto child) { return Cast<TSceneComponent>(child); }) | // no RTTI available, so use Cast instead of dynamic_cast
            std::views::transform([](auto child) { return static_cast<TReturnAsSceneComponent*>(child); }));
    }

    // specialization for AActor
    template <CSceneComponent TReturnAsSceneComponent = USceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByClass(const AActor* parent, UClass* uclass, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        USceneComponent* root = parent->GetRootComponent();
        std::vector<TReturnAsSceneComponent*> components;
        if (root) {
            SP_ASSERT(root->GetClass());
            if (root->GetClass()->IsChildOf(uclass)) {
                components.push_back(static_cast<TReturnAsSceneComponent*>(root));
            }
            if (include_all_descendants) {
                std::vector<TReturnAsSceneComponent*> children = getChildrenComponentsByClass<TReturnAsSceneComponent>(root, uclass, include_all_descendants);
                components.insert(components.end(), children.begin(), children.end()); // TODO: replace with components.append_range(children) in C++23
            }
        }
        return components;
    }

    // specialization for USceneComponent
    template <CSceneComponent TReturnAsSceneComponent = USceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByClass(const USceneComponent* parent, UClass* uclass, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        TArray<USceneComponent*> children_tarray;
        parent->GetChildrenComponents(include_all_descendants, children_tarray);
        std::vector<USceneComponent*> children = toStdVector(children_tarray);
        SP_ASSERT(!Std::contains(children, nullptr));
        SP_ASSERT(Std::all(Std::toVector<bool>(children | std::views::transform([](auto child) { return static_cast<bool>(child->GetClass()); })))); // cast to bool needed on Windows
        return Std::toVector<TReturnAsSceneComponent*>(
            children |
            std::views::filter([uclass](auto child) { return child->GetClass()->IsChildOf(uclass); }) |
            std::views::transform([](auto child) { return static_cast<TReturnAsSceneComponent*>(child); }));
    }

    //
    // Get children components by name or tag or type and return an std::map
    //

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByNameAsMap(
        const TParent* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants = true, bool return_null_if_not_found = true)
    {
        std::map<std::string, TReturnAsSceneComponent*> component_map;
        std::vector<std::string> unique_children_component_names = Std::unique(children_component_names);

        if (return_null_if_not_found) {
            component_map = Std::zip(
                unique_children_component_names,
                getChildrenComponentsByName<TParent, TSceneComponent, TReturnAsSceneComponent>(parent, unique_children_component_names, include_all_descendants, return_null_if_not_found));
        } else {
            std::vector<TSceneComponent*> components = getChildrenComponentsByName<TParent, TSceneComponent>(parent, unique_children_component_names, include_all_descendants, return_null_if_not_found);
            component_map = toMap<TReturnAsSceneComponent>(components);
        }

        return component_map;
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagAsMap(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return toMap<TReturnAsSceneComponent>(getChildrenComponentsByTag<TParent, TSceneComponent>(parent, tag, include_all_descendants));
    }
    
    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagAnyAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toMap<TReturnAsSceneComponent>(getChildrenComponentsByTagAny<TParent, TSceneComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagAllAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toMap<TReturnAsSceneComponent>(getChildrenComponentsByTagAll<TParent, TSceneComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTypeAsMap(const TParent* parent, bool include_all_descendants = true)
    {
        return toMap<TReturnAsSceneComponent>(getChildrenComponentsByType<TSceneComponent>(parent, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TReturnAsSceneComponent = USceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByClassAsMap(const TParent* parent, UClass* uclass, bool include_all_descendants = true)
    {
        return toMap<TReturnAsSceneComponent>(getChildrenComponentsByClass<TReturnAsSceneComponent>(parent, uclass, include_all_descendants));
    }

    //
    // Get child component by name or tag or type and return a pointer
    //

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByName(const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true)
    {
        bool return_null_if_not_found = false;
        return getItem(getChildrenComponentsByName<TParent, TSceneComponent, TReturnAsSceneComponent>(parent, {child_component_name}, include_all_descendants, return_null_if_not_found));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByTag(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByTag<TParent, TSceneComponent, TReturnAsSceneComponent>(parent, tag, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByTagAny(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByTagAny<TParent, TSceneComponent, TReturnAsSceneComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByTagAll(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByTagAll<TParent, TSceneComponent, TReturnAsSceneComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByType(const TParent* parent, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByType<TSceneComponent, TReturnAsSceneComponent>(parent, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TReturnAsSceneComponent = USceneComponent>
    static TReturnAsSceneComponent* getChildComponentByClass(const TParent* parent, UClass* uclass, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByClass<TReturnAsSceneComponent>(parent, uclass, include_all_descendants));
    }

    //
    // Helper functions for getting subsystem providers
    //

    template <CSubsystemProvider TSubsystemProvider>
    static TSubsystemProvider* getSubsystemProvider(const UObject* context)
    {
        SP_ASSERT(false);
        return nullptr;
    }

    template <>
    ULocalPlayer* getSubsystemProvider<ULocalPlayer>(const UObject* context)
    {
        SP_ASSERT(context);
        const UWorld* world = Cast<UWorld>(context); // no RTTI available, so use Cast instead of dynamic_cast
        SP_ASSERT(world);
        APlayerController* player_controller = world->GetFirstPlayerController();
        SP_ASSERT(player_controller);
        ULocalPlayer* local_player = player_controller->GetLocalPlayer();
        SP_ASSERT(local_player);
        return local_player;
    }

    //
    // Get and set actor and component stable names
    //

    static bool hasStableName(const AActor* actor);
    static bool hasStableName(const UActorComponent* component);

    static std::string getStableName(const AActor* actor);
    static std::string getStableName(const UActorComponent* component, bool include_actor_name = false);

    static void setStableName(const AActor* actor, const std::string& stable_name);
    #if WITH_EDITOR // defined in an auto-generated header
        static void requestUpdateStableName(const AActor* actor);
    #endif

    //
    // Get object tags
    //

    static std::vector<std::string> getTags(const AActor* actor);
    static std::vector<std::string> getTags(const UActorComponent* component);

    //
    // Helper functions for working with enums
    //

    // Enum output, enum input

    template <typename TEnum>
    static constexpr auto getConstEnumValue(const TEnum src)
    {
        return static_cast<std::underlying_type_t<TEnum>>(src); // needed to assign existing enum values to new enum values
    }

    template <typename TEnum>
    static int64_t getEnumValue(TEnum src)
    {
        return static_cast<std::underlying_type_t<TEnum>>(src);
    }

    template <> int64_t getEnumValue<int32>(int32 src) { return src; } // needed for getEnumValue(0)
    template <> int64_t getEnumValue<int64>(int64 src) { return src; } // needed for getEnumValue(uenum->GetValueByName(...))

    template <typename TDestEnum, typename TSrcEnum>
    static TDestEnum getEnumValueAs(TSrcEnum src)
    {
        return static_cast<TDestEnum>(getEnumValue(src));
    }

    // Enum output, string input

    template <CEnum TEnum>
    static TEnum getEnumValueFromString(const std::string& string)
    {
        UEnum* uenum = StaticEnum<TEnum>();
        return getEnumValueAs<TEnum>(uenum->GetValueByName(toFName(string)));
    }

    template <typename TDestEnum, CEnum TEnum>
    static TDestEnum getEnumValueFromStringAs(const std::string& string)
    {
        return getEnumValueAs<TDestEnum>(getEnumValueFromString<TEnum>(string));
    }

    template <CEnum TEnum>
    static auto getCombinedEnumFlagValueFromStrings(const std::vector<std::string>& strings)
    {
        TEnum combined_value = getEnumValueAs<TEnum>(0);
        for (auto& string : strings) {
            combined_value |= getEnumValueFromString<TEnum>(string);
        }
        return combined_value;
    }

    template <typename TDestEnum, CEnum TEnum>
    static auto getCombinedEnumFlagValueFromStringsAs(const std::vector<std::string>& strings)
    {
        return getEnumValueAs<TDestEnum>(getCombinedEnumFlagValueFromStrings<TEnum>(strings));
    }

    // String output, enum input

    template <CEnum TEnum>
    static std::string getStringFromEnumValue(TEnum src)
    {
        return getStringFromEnumValueAs<TEnum>(src);
    }

    template <CEnum TEnum, typename TSrcEnum>
    static std::string getStringFromEnumValueAs(TSrcEnum src)
    {
        UEnum* uenum = StaticEnum<TEnum>();
        return toStdString(uenum->GetNameStringByValue(getEnumValue(src)));
    }

    template <CEnum TEnum>
    static std::vector<std::string> getStringsFromCombinedEnumFlagValue(TEnum src)
    {
        return getStringsFromCombinedEnumFlagValueAs<TEnum>(src);
    }

    template <CEnum TEnum, typename TSrcEnum>
    static std::vector<std::string> getStringsFromCombinedEnumFlagValueAs(TSrcEnum src)
    {
        UEnum* uenum = StaticEnum<TEnum>();
        return Std::tokenize(toStdString(uenum->GetValueOrBitfieldAsString(getEnumValue(src))), "| ");
    }

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
            dest.push_back(data);
        }
        return dest;
    }

    template <typename TValue>
    static TArray<TValue> toTArray(const std::vector<TValue>& src)
    {
        TArray<TValue> dest;
        for (auto& data : src) {
            dest.Add(data);
        }
        return dest;
    }

    template <typename TDestValue, typename TValue>
    static TArray<TDestValue> toTArrayOf(const std::vector<TValue>& src)
    {
        TArray<TDestValue> dest;
        for (auto& data : src) {
            dest.Add(data);
        }
        return dest;
    }

    //
    // String functions
    //

    static std::string toStdString(const FString& str);
    static std::string toStdString(const FName& str);
    static std::string toStdString(const TCHAR* str);
    static FString toFString(const std::string& str);
    static FName toFName(const std::string& str);

private:

    //
    // Helper functions for finding actors and getting components
    //

    template <typename TReturnAsStableNameObject, typename TStableNameObject> requires
        CStableNameObject<TStableNameObject> && std::derived_from<TStableNameObject, TReturnAsStableNameObject>
    static std::map<std::string, TReturnAsStableNameObject*> toMap(const std::vector<TStableNameObject*>& objects)
    {
        return Std::toMap<std::string, TReturnAsStableNameObject*>(
            objects |
            std::views::filter([](auto object) { return hasStableName(object); }) |
            std::views::transform([](auto object) { return std::make_pair(getStableName(object), static_cast<TReturnAsStableNameObject*>(object)); }));
    }

    template <typename TValue>
    static const TValue& getItem(const std::vector<TValue>& vector)
    {
        SP_ASSERT(vector.size() == 1);
        return vector.at(0);
    }

    //
    // Helper functions for formatting container properties as strings in the same style as Unreal
    //

    static std::string getArrayPropertyValueAsFormattedString(
        const FProperty* inner_property, const std::vector<std::string>& inner_strings);

    static std::string getMapPropertyValueAsFormattedString(
        const FProperty* inner_key_property, const std::vector<std::string>& inner_key_strings,
        const FProperty* inner_value_property, const std::vector<std::string>& inner_value_strings);

    static std::string getQuoteStringForProperty(const FProperty* property);
};
