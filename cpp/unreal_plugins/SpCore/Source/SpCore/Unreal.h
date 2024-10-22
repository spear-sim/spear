//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

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
#include <Containers/UnrealString.h> // FString::operator*
#include <EngineUtils.h>             // TActorIterator
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // TCHAR
#include <Templates/Casts.h>
#include <UObject/Class.h>           // EIncludeSuperFlag, UClass, UStruct
#include <UObject/NameTypes.h>       // FName
#include <UObject/Object.h>          // UObject
#include <UObject/UnrealType.h>      // FProperty

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

class UWorld;

//
// Helper macros for working with enums
//

#define SP_DECLARE_ENUM_PROPERTY(TEnumType, Enum) \
    using TEnum = TEnumType;                       \
    static std::string getName() { return #Enum; } \
    TEnum getValue() const { return Enum; }        \
    void setValue(TEnum value) { Enum = value; }

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

template <typename TEnumStruct>
concept CEnumStruct =
    CStruct<TEnumStruct> &&
    requires(TEnumStruct enum_struct) {
        typename TEnumStruct::TEnum;
        { enum_struct.getName() } -> std::same_as<std::string>;
        { enum_struct.getValue() } -> std::same_as<typename TEnumStruct::TEnum>;
    };

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

//
// General-purpose functions for working with Unreal objects.
//

class SPCORE_API Unreal
{
public:
    Unreal() = delete;
    ~Unreal() = delete;

    //
    // Get and set object properties, uobject can't be const because we cast it to void*
    //

    static std::string getObjectPropertiesAsString(UObject* uobject);
    static std::string getObjectPropertiesAsString(void* value_ptr, const UStruct* ustruct);

    static void setObjectPropertiesFromString(UObject* uobject, const std::string& string);
    static void setObjectPropertiesFromString(void* value_ptr, const UStruct* ustruct, const std::string& string);

    //
    // Find property by name, get and set property values, uobject can't be const because we cast it to void*
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

    //
    // Find function by name, call function, world can't be const because we cast it to void*, uobject can't
    // be const because we call uobject->ProcessEvent(...) which is non-const, ufunction can't be const
    // because we call because we pass it to uobject->ProcessEvent(...) which expects non-const
    //

    static UFunction* findFunctionByName(const UClass* uclass, const std::string& function_name, EIncludeSuperFlag::Type include_super_flag = EIncludeSuperFlag::IncludeSuper);
    static std::map<std::string, std::string> callFunction(UWorld* world, UObject* uobject, UFunction* ufunction, const std::map<std::string, std::string>& args = {}, const std::string& world_context = "WorldContextObject");

    //
    // Find special struct by name. For this function to behave as expected, ASpSpecialStructActor must have
    // a UPROPERTY defined on it named TypeName_ of type TypeName.
    //

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
    // Find actors by name or tag or type and return an std::vector
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByName(const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsActor*> actors;
        std::map<std::string, TReturnAsActor*> actor_map = toMap(findActorsByType<TActor, TReturnAsActor>(world));
        if (return_null_if_not_found) {
            actors = Std::at(actor_map, actor_names, nullptr);
        } else {
            auto names_found_in_actor_map = Std::toVector<std::string>(actor_names | std::views::filter([&actor_map](const auto& name) { return Std::containsKey(actor_map, name); }));
            actors = Std::at(actor_map, names_found_in_actor_map);
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
        if (return_null_if_not_found) {
            return Std::zip(actor_names, findActorsByName<TActor, TReturnAsActor>(world, actor_names, return_null_if_not_found));
        } else {
            return toMap(findActorsByName<TActor, TReturnAsActor>(world, actor_names, return_null_if_not_found));
        }
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAsMap(const UWorld* world, const std::string& tag)
    {
        return toMap(findActorsByTagAny<TActor, TReturnAsActor>(world, {tag}));
    }
    
    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAnyAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        return toMap(findActorsByTagAny<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAllAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        return toMap(findActorsByTagAll<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTypeAsMap(const UWorld* world)
    {
        return toMap(findActorsByType<TActor, TReturnAsActor>(world));
    }

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByClassAsMap(const UWorld* world, UClass* uclass)
    {
        return toMap(findActorsByClass<TReturnAsActor>(world, uclass));
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
    static std::vector<TReturnAsComponent*> getComponentsByName(const AActor* actor, const std::vector<std::string>& component_names, bool include_from_child_actors = false, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsComponent*> components;
        std::map<std::string, TReturnAsComponent*> component_map = toMap(getComponentsByType<TComponent, TReturnAsComponent>(actor, include_from_child_actors));
        if (return_null_if_not_found) {
            components = Std::at(component_map, component_names, nullptr);
        } else {
            auto names_found_in_component_map =
                Std::toVector<std::string>(component_names | std::views::filter([&component_map](const auto& name) { return Std::containsKey(component_map, name); }));
            components = Std::at(component_map, names_found_in_component_map);
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
        if (return_null_if_not_found) {
            return Std::zip(component_names, getComponentsByName<TComponent, TReturnAsComponent>(actor, component_names, include_from_child_actors, return_null_if_not_found));
        } else {
            return toMap(getComponentsByName<TComponent, TReturnAsComponent>(actor, component_names, include_from_child_actors, return_null_if_not_found));
        }
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAsMap(const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByTagAny<TComponent, TReturnAsComponent>(actor, {tag}, include_from_child_actors));
    }
    
    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAnyAsMap(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByTagAny<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAllAsMap(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByTagAll<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTypeAsMap(const AActor* actor, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByType<TComponent, TReturnAsComponent>(actor, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByClassAsMap(const AActor* actor, UClass* uclass, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByClass<TReturnAsComponent>(actor, uclass, include_from_child_actors));
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

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByName(const TParent* parent, const std::vector<std::string>& scene_component_names, bool include_all_descendants = true, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsComponent*> components;
        std::map<std::string, TReturnAsComponent*> component_map = toMap(getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(parent, include_all_descendants));
        if (return_null_if_not_found) {
            components = Std::at(component_map, scene_component_names, nullptr);
        } else {
            auto names_found_in_component_map =
                Std::toVector<std::string>(scene_component_names | std::views::filter([&component_map](const auto& name) { return Std::containsKey(component_map, name); }));
            components = Std::at(component_map, names_found_in_component_map);
        }
        return components;
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByTag(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return getChildrenComponentsByTagAny<TParent, TSceneComponent, TReturnAsComponent>(parent, {tag}, include_all_descendants);
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByTagAny(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return Std::toVector<TReturnAsComponent*>(
            getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(parent, include_all_descendants) |
            std::views::filter([&tags](auto component) { return Std::any(Std::contains(getTags(component), tags)); }));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByTagAll(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return Std::toVector<TReturnAsComponent*>(
            getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(parent, include_all_descendants) |
            std::views::filter([&tags](auto component) { return Std::all(Std::contains(getTags(component), tags)); }));
    }

    // specialization for AActor
    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByType(const AActor* parent, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        USceneComponent* root = parent->GetRootComponent();
        std::vector<TReturnAsComponent*> components;
        if (Cast<TSceneComponent>(root)) { // no RTTI available, so use Cast instead of dynamic_cast
            components.push_back(static_cast<TReturnAsComponent*>(root));
        }
        if (root && include_all_descendants) {
            std::vector<TReturnAsComponent*> children = getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(root, include_all_descendants);
            components.insert(components.end(), children.begin(), children.end()); // TODO: replace with components.append_range(children) in C++23
        }
        return components;
    }

    // specialization for USceneComponent
    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByType(const USceneComponent* parent, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        TArray<USceneComponent*> children_tarray;
        parent->GetChildrenComponents(include_all_descendants, children_tarray);
        std::vector<USceneComponent*> children = toStdVector(children_tarray);
        SP_ASSERT(!Std::contains(children, nullptr));
        return Std::toVector<TReturnAsComponent*>(
            children |
            std::views::filter([](auto child) { return Cast<TSceneComponent>(child); }) | // no RTTI available, so use Cast instead of dynamic_cast
            std::views::transform([](auto child) { return static_cast<TReturnAsComponent*>(child); }));
    }

    // specialization for AActor
    template <CSceneComponent TReturnAsComponent = USceneComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByClass(const AActor* parent, UClass* uclass, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        USceneComponent* root = parent->GetRootComponent();
        std::vector<TReturnAsComponent*> components;
        if (root) {
            SP_ASSERT(root->GetClass());
            if (root->GetClass()->IsChildOf(uclass)) {
                components.push_back(static_cast<TReturnAsComponent*>(root));
            }
            if (include_all_descendants) {
                std::vector<TReturnAsComponent*> children = getChildrenComponentsByClass<TReturnAsComponent>(root, uclass, include_all_descendants);
                components.insert(components.end(), children.begin(), children.end()); // TODO: replace with components.append_range(children) in C++23
            }
        }
        return components;
    }

    // specialization for USceneComponent
    template <CSceneComponent TReturnAsComponent = USceneComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByClass(const USceneComponent* parent, UClass* uclass, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        TArray<USceneComponent*> children_tarray;
        parent->GetChildrenComponents(include_all_descendants, children_tarray);
        std::vector<USceneComponent*> children = toStdVector(children_tarray);
        SP_ASSERT(!Std::contains(children, nullptr));
        SP_ASSERT(Std::all(Std::toVector<bool>(children | std::views::transform([](auto child) { return static_cast<bool>(child->GetClass()); })))); // cast to bool needed on Windows
        return Std::toVector<TReturnAsComponent*>(
            children |
            std::views::filter([uclass](auto child) { return child->GetClass()->IsChildOf(uclass); }) |
            std::views::transform([](auto child) { return static_cast<TReturnAsComponent*>(child); }));
    }

    //
    // Get children components by name or tag or type and return an std::map
    //

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByNameAsMap(const TParent* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants = true, bool return_null_if_not_found = true)
    {
        if (return_null_if_not_found) {
            return Std::zip(children_component_names, getChildrenComponentsByName<TParent, TSceneComponent, TReturnAsComponent>(parent, children_component_names, include_all_descendants, return_null_if_not_found));
        } else {
            return toMap(getChildrenComponentsByName<TParent, TSceneComponent, TReturnAsComponent>(parent, children_component_names, include_all_descendants, return_null_if_not_found));
        }
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByTagAsMap(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByTagAny<TParent, TSceneComponent, TReturnAsComponent>(parent, {tag}, include_all_descendants));
    }
    
    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByTagAnyAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByTagAny<TParent, TSceneComponent, TReturnAsComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByTagAllAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByTagAll<TParent, TSceneComponent, TReturnAsComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByTypeAsMap(const TParent* parent, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(parent, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TReturnAsComponent = USceneComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByClassAsMap(const TParent* parent, UClass* uclass, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByClass<TReturnAsComponent>(parent, uclass, include_all_descendants));
    }

    //
    // Get child component by name or tag or type and return a pointer
    //

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByName(const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true)
    {
        bool return_null_if_not_found = false;
        return getItem(getChildrenComponentsByName<TParent, TSceneComponent, TReturnAsComponent>(parent, {child_component_name}, include_all_descendants, return_null_if_not_found));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByTag(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByTag<TParent, TSceneComponent, TReturnAsComponent>(parent, tag, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByTagAny(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByTagAny<TParent, TSceneComponent, TReturnAsComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByTagAll(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByTagAll<TParent, TSceneComponent, TReturnAsComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByType(const TParent* parent, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(parent, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TReturnAsComponent = USceneComponent>
    static TReturnAsComponent* getChildComponentByClass(const TParent* parent, UClass* uclass, bool include_all_descendants = true)
    {
        return getItem(getChildrenComponentsByClass<TReturnAsComponent>(parent, uclass, include_all_descendants));
    }

    //
    // Get and set actor and component stable names
    //

    static bool hasStableName(const AActor* actor);
    static bool hasStableName(const UActorComponent* component);

    static std::string getStableName(const AActor* actor);
    static std::string getStableName(const CComponent auto* actor_component, bool include_actor_name = false)
    {
        SP_ASSERT(actor_component);
        std::string component_name = toStdString(actor_component->GetName());
        if (include_actor_name) {
            AActor* actor = actor_component->GetOwner();
            SP_ASSERT(actor);
            component_name = getStableName(actor) + ":" + component_name;
        }
        return component_name;
    }

    static std::string getStableName(const CSceneComponent auto* scene_component, bool include_actor_name = false)
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
            component_name = getStableName(actor) + ":" + component_name;
        }
        return component_name;
    }

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

    template <typename TEnum>
    static constexpr auto getConstEnumValue(const TEnum src)
    {
        return static_cast<std::underlying_type_t<TEnum>>(src);
    }

    static auto getEnumValue(int src)
    {
        return src;
    }

    template <typename TEnum>
    static auto getEnumValue(TEnum src)
    {
        return static_cast<std::underlying_type_t<TEnum>>(src);
    }

    template <CEnumStruct TEnumStruct>
    static auto getEnumValue(TEnumStruct src)
    {
        using TEnum = typename TEnumStruct::TEnum;

        return static_cast<std::underlying_type_t<TEnum>>(src.getValue());
    }

    template <typename TDestEnum, typename TSrcEnum>
    static TDestEnum getEnumValueAs(TSrcEnum src)
    {
        return static_cast<TDestEnum>(getEnumValue(src));
    }

    template <CEnumStruct TEnumStruct, typename TSrcEnum>
    static std::string getEnumValueAsString(TSrcEnum src)
    {
        using TEnum = typename TEnumStruct::TEnum;

        TEnumStruct enum_struct;
        enum_struct.setValue(getEnumValueAs<TEnum>(src));
        PropertyDesc property_desc = findPropertyByName(&enum_struct, TEnumStruct::StaticStruct(), enum_struct.getName());
        return getPropertyValueAsString(property_desc);
    }

    template <CEnumStruct TEnumStruct, typename TDestEnum>
    static TDestEnum getEnumValueFromString(const std::string& string)
    {
        TEnumStruct enum_struct;
        PropertyDesc property_desc = findPropertyByName(&enum_struct, TEnumStruct::StaticStruct(), enum_struct.getName());
        setPropertyValueFromString(property_desc, string);
        return getEnumValueAs<TDestEnum>(enum_struct);
    }

    template <CEnumStruct TEnumStruct>
    static auto combineEnumFlagStrings(const std::vector<std::string>& enum_flag_strings)
    {
        using TEnum = typename TEnumStruct::TEnum;

        TEnum combined_value = getEnumValueAs<TEnum>(0);
        for (auto& enum_flag_string : enum_flag_strings) {
            combined_value |= getEnumValueFromString<TEnumStruct, TEnum>(enum_flag_string);
        }
        return combined_value;
    }

    //
    // Helper function to find special structs
    //

    static UStruct* findSpecialStructByName(const std::string& struct_name);

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

    template <typename TStableNameObject> requires CStableNameObject<TStableNameObject>
    static std::map<std::string, TStableNameObject*> toMap(const std::vector<TStableNameObject*>& objects)
    {
        return Std::toMap<std::string, TStableNameObject*>(
            objects |
            std::views::filter([](auto object) { return hasStableName(object); }) |
            std::views::transform([](auto object) { return std::make_pair(getStableName(object), object); }));
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
};
