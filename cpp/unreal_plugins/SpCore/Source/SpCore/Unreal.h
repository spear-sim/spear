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

template <typename TObject>
concept CObject = std::derived_from<TObject, UObject>;

template <typename TStruct>
concept CStruct =
    CObject<TStruct> &&
    requires() {
        { TStruct::StaticStruct() } -> std::same_as<UStruct*>;
    };

template <typename TComponent>
concept CComponent = CObject<TComponent> && std::derived_from<TComponent, UActorComponent>;

template <typename TSceneComponent>
concept CSceneComponent = CComponent<TSceneComponent> && std::derived_from<TSceneComponent, USceneComponent>;

template <typename TActor>
concept CActor = CObject<TActor> && std::derived_from<TActor, AActor>;

template <typename TParent>
concept CParent = CActor<TParent> || CSceneComponent<TParent>;

class SPCORE_API Unreal
{
public:
    Unreal() = delete;
    ~Unreal() = delete;

    // 
    // Find actors unconditionally and return an std::vector or an std::map
    //

    static std::vector<AActor*> findActors(const UWorld* world);
    static std::map<std::string, AActor*> findActorsAsMap(const UWorld* world);


    //
    // Get components unconditionally and return an std::vector or an std::map
    //

    std::vector<UActorComponent*> getComponents(const AActor* actor);
    std::map<std::string, UActorComponent*> getComponentsAsMap(const AActor* actor);

    //
    // Get children components unconditionally and return an std::vector or an std::map
    //

    std::vector<USceneComponent*> getChildrenComponents(const USceneComponent* parent, bool include_all_descendants = true);
    std::map<std::string, USceneComponent*> getChildrenComponentsAsMap(const USceneComponent* parent, bool include_all_descendants = true);

    //
    // Find struct by name
    //

    static UStruct* findStructByName(const std::string& name);

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
    static std::map<std::string, std::string> callFunction(UObject* uobject, UFunction* ufunction);
    static std::map<std::string, std::string> callFunction(UObject* uobject, UFunction* ufunction, const std::map<std::string, std::string>& args);

    //
    // Convert a vector of objects into a map using each object's stable name as its key
    //

    template <typename TObject> requires CActor<TObject> || CComponent<TObject>
    static std::map<std::string, TObject*> getObjectsAsMap(const std::vector<TObject*>& objects)
    {
        return Std::toMap<std::string, TObject*>(
            objects |
            std::views::filter([](const auto& object) { return hasStableName(object); }) |
            std::views::transform([](const auto& object) { return std::make_pair(getStableName(object), object); }));
    }

    //
    // Get and set actor and component stable names
    //

    static bool hasStableName(const AActor* actor);
    static bool hasStableName(const UActorComponent* component);

    static std::string getStableName(const AActor* actor);
    static std::string getStableName(const CComponent auto* actor_component, bool include_stable_actor_name = false)
    {
        SP_ASSERT(actor_component);
        std::string component_name = toStdString(actor_component->GetName());
        if (include_stable_actor_name) {
            AActor* actor = actor_component->GetOwner();
            SP_ASSERT(actor);
            component_name = getStableName(actor) + ":" + component_name;
        }
        return component_name;
    }

    static std::string getStableName(const CSceneComponent auto* scene_component, bool include_stable_actor_name = false)
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
        SP_ASSERT(parent);
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
    static TReturnAsComponent* createComponentOutsideOwnerConstructor(AActor* owner, const std::string& name)
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
    // Find actors by name or tag or type and return an std::vector
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByName(const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsActor*> actors;
        std::map<std::string, TReturnAsActor*> actor_map = getObjectsAsMap(findActorsByType<TActor, TReturnAsActor>(world));
        if (return_null_if_not_found) {
            actors = Std::at(actor_map, names, nullptr);
        } else {
            auto names_found_in_actor_map = Std::toVector<std::string>(names | std::views::filter([&actor_map](const auto& name) { return Std::containsKey(actor_map, name); }));
            actors = Std::at(actor_map, names_found_in_actor_map);
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
        return Std::toVector<TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::filter([&tags](auto actor) { return Std::any(Std::contains(getTags(actor), tags)); }));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTagAll(const UWorld* world, const std::vector<std::string>& tags)
    {
        return Std::toVector<TReturnAsActor*>(
            findActorsByType<TActor, TReturnAsActor>(world) |
            std::views::filter([&tags](auto actor) { return Std::all(Std::contains(getTags(actor), tags)); }));
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
        return getObjectsAsMap(findActorsByName<TActor, TReturnAsActor>(world, names));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAsMap(const UWorld* world, const std::string& tag)
    {
        return getObjectsAsMap(findActorsByTagAny<TActor, TReturnAsActor>(world, {tag}));
    }
    
    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAnyAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        return getObjectsAsMap(findActorsByTagAny<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAllAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        return getObjectsAsMap(findActorsByTagAll<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTypeAsMap(const UWorld* world)
    {
        return getObjectsAsMap(findActorsByType<TActor, TReturnAsActor>(world));
    }

    //
    // Find actor by name or tag or type and return a pointer
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByName(const UWorld* world, const std::string& name, bool assert_if_not_found = true)
    {
        bool assert_if_multiple_found = true;
        bool return_null_if_not_found = false;
        return getItem(findActorsByName<TActor, TReturnAsActor>(world, {name}, return_null_if_not_found), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTag(const UWorld* world, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(findActorsByTag<TActor, TReturnAsActor>(world, tag), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTagAny(const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(findActorsByTagAny<TActor, TReturnAsActor>(world, tags), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTagAll(const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(findActorsByTagAll<TActor, TReturnAsActor>(world, tags), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByType(const UWorld* world, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(findActorsByType<TActor, TReturnAsActor>(world), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    //
    // Get components by name or tag or type and return an std::vector
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByName(const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsComponent*> components;
        std::map<std::string, TReturnAsComponent*> component_map = getObjectsAsMap(getComponentsByType<TComponent, TReturnAsComponent>(actor));
        if (return_null_if_not_found) {
            components = Std::at(component_map, names, nullptr);
        } else {
            auto names_found_in_component_map =
                Std::toVector<std::string>(names | std::views::filter([&component_map](const auto& name) { return Std::containsKey(component_map, name); }));
            components = Std::at(component_map, names_found_in_component_map);
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
        return Std::toVector<TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor) |
            std::views::filter([&tags](auto component) { return Std::any(Std::contains(getTags(component), tags)); }));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagAll(const AActor* actor, const std::vector<std::string>& tags)
    {
        return Std::toVector<TReturnAsComponent*>(
            getComponentsByType<TComponent, TReturnAsComponent>(actor) |
            std::views::filter([&tags](auto component) { return Std::all(Std::contains(getTags(component), tags)); }));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByType(const AActor* actor)
    {
        SP_ASSERT(actor);
        TArray<TComponent*> components_tarray;
        actor->GetComponents<TComponent>(components_tarray);
        std::vector<TReturnAsComponent*> components = toStdVectorOf<TReturnAsComponent*>(components_tarray);
        SP_ASSERT(!Std::contains(components, nullptr));
        return components;
    }

    //
    // Get components by name or tag or type and return an std::map
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByNameAsMap(const AActor* actor, const std::vector<std::string>& names)
    {
        return getObjectsAsMap(getComponentsByName<TComponent, TReturnAsComponent>(actor, names));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAsMap(const AActor* actor, const std::string& tag)
    {
        return getObjectsAsMap(getComponentsByTagAny<TComponent, TReturnAsComponent>(actor, {tag}));
    }
    
    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAnyAsMap(const AActor* actor, const std::vector<std::string>& tags)
    {
        return getObjectsAsMap(getComponentsByTagAny<TComponent, TReturnAsComponent>(actor, tags));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAllAsMap(const AActor* actor, const std::vector<std::string>& tags)
    {
        return getObjectsAsMap(getComponentsByTagAll<TComponent, TReturnAsComponent>(actor, tags));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTypeAsMap(const AActor* actor)
    {
        return getObjectsAsMap(getComponentsByType<TComponent, TReturnAsComponent>(actor));
    }

    //
    // Get component by name or tag or type and return a pointer
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByName(const AActor* actor, const std::string& name, bool assert_if_not_found = true)
    {
        bool assert_if_multiple_found = true;
        bool return_null_if_not_found = false;
        return getItem(getComponentsByName<TComponent, TReturnAsComponent>(actor, {name}, return_null_if_not_found), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTag(const AActor* actor, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getComponentsByTag<TComponent, TReturnAsComponent>(actor, tag), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTagAny(const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getComponentsByTagAny<TComponent, TReturnAsComponent>(actor, tags), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTagAll(const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getComponentsByTagAll<TComponent, TReturnAsComponent>(actor, tags), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByType(const AActor* actor, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getComponentsByType<TComponent, TReturnAsComponent>(actor), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    //
    // Get children components by name or tag or type and return an std::vector
    //

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByName(const TParent* parent, const std::vector<std::string>& names, bool include_all_descendants = true, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsComponent*> components;
        std::map<std::string, TReturnAsComponent*> component_map = getObjectsAsMap(getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(parent, include_all_descendants));
        if (return_null_if_not_found) {
            components = Std::at(component_map, names, nullptr);
        } else {
            auto names_found_in_component_map =
                Std::toVector<std::string>(names | std::views::filter([&component_map](const auto& name) { return Std::containsKey(component_map, name); }));
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

    // specialization for AActor*
    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByType(const AActor* parent, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        USceneComponent* root = parent->GetRootComponent();
        std::vector<TReturnAsComponent*> components;
        if (root && dynamic_cast<TSceneComponent*>(root)) {
            components.push_back(root);
        }
        if (root && include_all_descendants) {
            std::vector<TReturnAsComponent*> children = getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(root, include_all_descendants);
            components.insert(components.end(), children.begin(), children.end()); // TODO: replace with components.append_range(children)
        }
        return components;
    }

    // specialization for USceneComponent*
    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getChildrenComponentsByType(const USceneComponent* parent, bool include_all_descendants = true)
    {
        SP_ASSERT(parent);
        TArray<USceneComponent*> children_tarray;
        parent->GetChildrenComponents(include_all_descendants, children_tarray);
        std::vector<USceneComponent*> children = toStdVector(children_tarray);
        SP_ASSERT(!Std::contains(children, nullptr));
        return Std::toVector<TReturnAsComponent*>(children | std::views::filter([](auto child) { return dynamic_cast<TSceneComponent*>(child); }));
    }

    //
    // Get children components by name or tag or type and return an std::map
    //

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByNameAsMap(const TParent* parent, const std::vector<std::string>& names, bool include_all_descendants = true)
    {
        return getObjectsAsMap(getChildrenComponentsByName<TParent, TSceneComponent, TReturnAsComponent>(parent, names, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByTagAsMap(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return getObjectsAsMap(getChildrenComponentsByTagAny<TParent, TSceneComponent, TReturnAsComponent>(parent, {tag}, include_all_descendants));
    }
    
    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByTagAnyAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return getObjectsAsMap(getChildrenComponentsByTagAny<TParent, TSceneComponent, TReturnAsComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByTagAllAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return getObjectsAsMap(getChildrenComponentsByTagAll<TParent, TSceneComponent, TReturnAsComponent>(parent, tags, include_all_descendants));
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getChildrenComponentsByTypeAsMap(const TParent* parent, bool include_all_descendants = true)
    {
        return getObjectsAsMap(getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(parent, include_all_descendants));
    }

    //
    // Get child component by name or tag or type and return a pointer
    //

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByName(const TParent* parent, const std::string& name, bool include_all_descendants = true, bool assert_if_not_found = true)
    {
        bool assert_if_multiple_found = true;
        bool return_null_if_not_found = false;
        return getItem(getChildrenComponentsByName<TParent, TSceneComponent, TReturnAsComponent>(parent, {name}, include_all_descendants, return_null_if_not_found), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByTag(const TParent* parent, const std::string& tag, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getChildrenComponentsByTag<TParent, TSceneComponent, TReturnAsComponent>(parent, tag, include_all_descendants), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByTagAny(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getChildrenComponentsByTagAny<TParent, TSceneComponent, TReturnAsComponent>(parent, tags, include_all_descendants), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByTagAll(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getChildrenComponentsByTagAll<TParent, TSceneComponent, TReturnAsComponent>(parent, tags, include_all_descendants), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

    template <CParent TParent, CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsComponent>
    static TReturnAsComponent* getChildComponentByType(const TParent* parent, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return getItem(getChildrenComponentsByType<TSceneComponent, TReturnAsComponent>(parent, include_all_descendants), nullptr, assert_if_not_found, assert_if_multiple_found);
    }

private:

    //
    // Helper functions for finding actors and getting components
    //

    template <typename TValue>
    static const TValue& getItem(const std::vector<TValue>& vector, void* default_value, bool assert_if_size_is_zero, bool assert_if_size_is_greater_than_one)
    {
        return getItem(vector, reinterpret_cast<TValue>(default_value), assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <typename TValue>
    static const TValue& getItem(const std::vector<TValue>& vector, const TValue& default_value, bool assert_if_size_is_zero, bool assert_if_size_is_greater_than_one)
    {
        if (assert_if_size_is_zero) {
            SP_ASSERT(vector.size() != 0);
        }
        if (assert_if_size_is_greater_than_one) {
            SP_ASSERT(vector.size() <= 1);
        }
        if (vector.size() == 0) {
            return default_value;
        } else {
            return vector.at(0);
        }
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
