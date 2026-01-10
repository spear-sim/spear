//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts> // std::derived_from
#include <map>
#include <ranges>   // std::views::drop, std::views::filter, std::views::transform
#include <string>
#include <utility>  // std::make_pair, std::pair
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>
#include <Containers/UnrealString.h> // FString::operator*
#include <Dom/JsonValue.h>
#include <EngineUtils.h>             // TActorIterator
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // SPCORE_API
#include <Templates/Casts.h>
#include <Templates/SharedPointer.h> // TSharedPtr
#include <UObject/Class.h>           // EIncludeSuperFlag
#include <UObject/Object.h>          // UObject
#include <UObject/ObjectMacros.h>    // EPropertyFlags
#include <UObject/Script.h>          // EFunctionFlags
#include <UObject/UnrealType.h>      // EFieldIterationFlags, FProperty, TFieldIterator
#include <UObject/UObjectGlobals.h>  // NewObject
#include <UObject/UObjectIterator.h>

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

class UClass;
class UFunction;
class UStruct;
class UScriptStruct;
class UWorld;

//
// SpPropertyDesc and SpPropertyValue encapsulate the data needed to get and set a UPROPERTY on a specific UObject.
//

struct SpPropertyDesc
{
    FProperty* property_ = nullptr;
    void* value_ptr_ = nullptr;
    std::string type_id_;
};

struct SpPropertyValue
{
    std::string value_;
    std::string type_id_;
};

//
// General-purpose utility functions for working with Unreal objects. The functions here are higher-level
// than the ones in the Unreal class.
//

class SPCORE_API UnrealUtils
{
public:
    UnrealUtils() = delete;
    ~UnrealUtils() = delete;

    //
    // Find static structs
    //

    static std::vector<UScriptStruct*> findStaticStructs();
    static std::map<std::string, UScriptStruct*> findStaticStructsAsMap();

    //
    // Get static classes
    //

    static std::vector<UClass*> findStaticClasses();
    static std::map<std::string, UClass*> findStaticClassesAsMap();

    static std::vector<UClass*> getDerivedClasses(UClass* uclass, bool recursive = true);
    static std::map<std::string, UClass*> getDerivedClassesAsMap(UClass* uclass, bool recursive = true);

    //
    // Find functions
    //

    static std::vector<UFunction*> findFunctions(const UClass* uclass, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::map<std::string, UFunction*> findFunctionsAsMap(const UClass* uclass, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);

    static std::vector<UFunction*> findFunctionsByName(const UClass* uclass, const std::string& function_name, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::vector<UFunction*> findFunctionsByFlagsAny(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::vector<UFunction*> findFunctionsByFlagsAll(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);

    static std::map<std::string, UFunction*> findFunctionsByNameAsMap(const UClass* uclass, const std::string& function_name, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::map<std::string, UFunction*> findFunctionsByFlagsAnyAsMap(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::map<std::string, UFunction*> findFunctionsByFlagsAllAsMap(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);

    static UFunction* findFunctionByName(const UClass* uclass, const std::string& function_name, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static UFunction* findFunctionByFlagsAny(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static UFunction* findFunctionByFlagsAll(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);

    //
    // Call function (could be declared in Unreal.h, but the implementation depends on SpPropertyDesc, so we declare here for clean interface layering)
    //

    static std::map<std::string, SpPropertyValue> callFunction(
        const UWorld* world,
        UObject* uobject,
        UFunction* ufunction,
        const std::map<std::string, std::string>& args = {},
        const std::string& world_context_object = "WorldContextObject");

    //
    // Find properties (could be templated but we choose to explicitly instantiate for readability)
    //

    static std::vector<FProperty*> findProperties(const UStruct* ustruct, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::vector<FProperty*> findProperties(const UFunction* ufunction, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);

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
    // Get children components unconditionally and return an std::vector or an std::map (could be templated but we choose to explicitly instantiate for readability)
    //

    static std::vector<USceneComponent*> getChildrenComponents(const AActor* parent, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponents(const USceneComponent* parent, bool include_all_descendants = true);

    static std::map<std::string, USceneComponent*> getChildrenComponentsAsMap(const AActor* parent, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsAsMap(const USceneComponent* parent, bool include_all_descendants = true);

    //
    // Get and set object properties
    //

    static std::string getObjectPropertiesAsString(const UObject* uobject);
    static std::string getObjectPropertiesAsString(const void* value_ptr, const UStruct* ustruct);

    static void setObjectPropertiesFromString(UObject* uobject, const std::string& string);
    static void setObjectPropertiesFromString(void* value_ptr, const UStruct* ustruct, const std::string& string);

    //
    // Find property by name, get and set property values, uobject can't be const because we cast it to
    // (non-const) void*, value_ptr can't be const because we assign to SpPropertyDesc::value_ptr_.
    //

    static SpPropertyDesc findPropertyByName(UObject* uobject, const std::string& property_name);
    static SpPropertyDesc findPropertyByName(void* value_ptr, const UStruct* ustruct, const std::string& property_name);

    static SpPropertyValue getPropertyValueAsString(const SpPropertyDesc& property_desc);
    static void setPropertyValueFromString(const SpPropertyDesc& property_desc, const std::string& string);
    static void setPropertyValueFromJsonValue(const SpPropertyDesc& property_desc, TSharedPtr<FJsonValue> json_value);

    //
    // Get and set actor and component stable names
    //

    static bool hasStableName(const AActor* actor);
    static bool hasStableName(const UActorComponent* component);

    static std::string tryGetStableName(const AActor* actor);
    static std::string getStableName(const AActor* actor);
    static std::string getStableName(const UActorComponent* component, bool include_actor_name = false, bool actor_must_have_stable_name = true);
    static void setStableName(const AActor* actor, const std::string& stable_name);

    #if WITH_EDITOR // defined in an auto-generated header
        static void requestAddStableNameActor(AActor* actor);
        static void requestRemoveStableNameActor(AActor* actor);
        static void requestUpdateStableName(AActor* actor);
        static void requestUpdateAllStableNameActors(const UWorld* world);
    #endif

    //
    // Find objects
    //

    static std::vector<UObject*> findObjects();

    //
    // Find static structs by name or type and return an std::vector
    //

    template <CStaticStruct TStaticStruct, CStaticStruct TReturnAsStaticStruct = TStaticStruct> requires
        std::derived_from<TStaticStruct, TReturnAsStaticStruct>
    static std::vector<TReturnAsStaticStruct*> findStaticStructsByName(const std::string& static_struct_name)
    {
        return findStaticStructsByName<TReturnAsStaticStruct>(TStaticStruct::StaticClass(), static_struct_name);
    }

    template <CStaticStruct TStaticStruct, CStaticStruct TReturnAsStaticStruct = TStaticStruct> requires
        std::derived_from<TStaticStruct, TReturnAsStaticStruct>
    static std::vector<TReturnAsStaticStruct*> findStaticStructsByType()
    {
        return findStaticStructsByClass<TReturnAsStaticStruct>(TStaticStruct::StaticClass());
    }

    // UClass* variants

    template <CStaticStruct TReturnAsStaticStruct = UStruct>
    static std::vector<TReturnAsStaticStruct*> findStaticStructsByName(UClass* uclass, const std::string& static_struct_name)
    {
        return Std::toVector<TReturnAsStaticStruct*>(
            findObjectsByClass<TReturnAsStaticStruct>(uclass) |
            std::views::filter([&static_struct_name](auto static_struct) { return Unreal::getTypeAsString(static_struct) == static_struct_name; }));
    }

    template <CStaticStruct TReturnAsStaticStruct = UStruct>
    static std::vector<TReturnAsStaticStruct*> findStaticStructsByClass(UClass* uclass)
    {
        return findObjectsByClass<TReturnAsStaticStruct>(uclass);
    }

    //
    // Find static structs by name or type and return an std::map
    //

    template <CStaticStruct TStaticStruct, CStaticStruct TReturnAsStaticStruct = TStaticStruct> requires
        std::derived_from<TStaticStruct, TReturnAsStaticStruct>
    static std::map<std::string, TReturnAsStaticStruct*> findStaticStructsByNameAsMap(const std::string& static_struct_name)
    {
        return toMap(findStaticStructsByName<TStaticStruct, TReturnAsStaticStruct>(static_struct_name));
    }

    template <CStaticStruct TStaticStruct, CStaticStruct TReturnAsStaticStruct = TStaticStruct> requires
        std::derived_from<TStaticStruct, TReturnAsStaticStruct>
    static std::map<std::string, TReturnAsStaticStruct*> findStaticStructsByTypeAsMap()
    {
        return toMap(findStaticStructsByType<TStaticStruct, TReturnAsStaticStruct>());
    }

    // UClass* variants

    template <CStaticStruct TReturnAsStaticStruct = UStruct>
    static std::map<std::string, TReturnAsStaticStruct*> findStaticStructsByNameAsMap(UClass* uclass, const std::string& static_struct_name)
    {
        return toMap(findStaticStructsByName<TReturnAsStaticStruct>(uclass, static_struct_name));
    }

    template <CStaticStruct TReturnAsStaticStruct = UStruct>
    static std::map<std::string, TReturnAsStaticStruct*> findStaticStructsByClassAsMap(UClass* uclass)
    {
        return toMap(findStaticStructsByClass<TReturnAsStaticStruct>(uclass));
    }

    //
    // Find static struct by name or type and return a pointer
    //

    template <CStaticStruct TStaticStruct, CStaticStruct TReturnAsStaticStruct = TStaticStruct> requires
        std::derived_from<TStaticStruct, TReturnAsStaticStruct>
    static TReturnAsStaticStruct* findStaticStructByName(const std::string& static_struct_name)
    {
        return toItem(findStaticStructsByName<TStaticStruct>(static_struct_name));
    }

    template <CStaticStruct TStaticStruct, CStaticStruct TReturnAsStaticStruct = TStaticStruct> requires
        std::derived_from<TStaticStruct, TReturnAsStaticStruct>
    static TReturnAsStaticStruct* findStaticStructByType()
    {
        return toItem(findStaticStructsByType<TStaticStruct>());
    }

    // UClass* variants

    template <CStaticStruct TReturnAsStaticStruct = UStruct>
    static TReturnAsStaticStruct* findStaticStructByName(UClass* uclass, const std::string& static_struct_name)
    {
        return toItem(findStaticStructsByName(uclass, static_struct_name));
    }

    template <CStaticStruct TReturnAsStaticStruct = UStruct>
    static TReturnAsStaticStruct* findStaticStructByClass(UClass* uclass)
    {
        return toItem(findStaticStructsByClass(uclass));
    }

    //
    // Find properties
    //

    template <CPropertyContainer TPropertyContainer>
    static std::vector<FProperty*> findPropertiesAll(const TPropertyContainer* property_container, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default)
    {
        SP_ASSERT(property_container);
        std::vector<FProperty*> properties;
        for (TFieldIterator<FProperty> itr(property_container, field_iteration_flags); itr; ++itr) {
            FProperty* property = *itr;
            properties.push_back(property);
        }
        return properties;
    }

    template <CPropertyContainer TPropertyContainer>
    static std::vector<FProperty*> findPropertiesByFlagsAny(const TPropertyContainer* property_container, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default)
    {
        return Std::toVector<FProperty*>(
            findPropertiesAll(property_container, field_iteration_flags) |
            std::views::filter([property_flags](auto property) { return property->HasAnyPropertyFlags(property_flags); }));
    }

    template <CPropertyContainer TPropertyContainer>
    static std::vector<FProperty*> findPropertiesByFlagsAll(const TPropertyContainer* property_container, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default)
    {
        return Std::toVector<FProperty*>(
            findPropertiesAll(property_container, field_iteration_flags) |
            std::views::filter([property_flags](auto property) { return property->HasAllPropertyFlags(property_flags); }));
    }

    template <CPropertyContainer TPropertyContainer>
    static std::map<std::string, FProperty*> findPropertiesAsMap(const TPropertyContainer* property_container, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default)
    {
        return toMap(findPropertiesAll(property_container, field_iteration_flags));
    }

    template <CPropertyContainer TPropertyContainer>
    static std::map<std::string, FProperty*> findPropertiesByFlagsAnyAsMap(const TPropertyContainer* property_container, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default)
    {
        return toMap(findPropertiesByFlagsAny(property_container, property_flags, field_iteration_flags));
    }

    template <CPropertyContainer TPropertyContainer>
    static std::map<std::string, FProperty*> findPropertiesByFlagsAllAsMap(const TPropertyContainer* property_container, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default)
    {
        return toMap(findPropertiesByFlagsAll(property_container, property_flags, field_iteration_flags));
    }

    //
    // Find actor and component and return both
    //

    template <CActor TActor = AActor, CComponent TComponent = UActorComponent, CActor TReturnAsActor = TActor, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TActor, TReturnAsActor> && std::derived_from<TComponent, TReturnAsComponent>
    static std::pair<TReturnAsActor*, TReturnAsComponent*> findActorAndComponentByName(const UWorld* world, const AActor* owner, const std::string& path)
    {
        bool get_component_by_path = false;
        return findActorAndComponent<TActor, TComponent, TReturnAsActor, TReturnAsComponent>(world, owner, path, get_component_by_path);
    }

    template <CActor TActor = AActor, CComponent TComponent = UActorComponent, CActor TReturnAsActor = TActor, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TActor, TReturnAsActor> && std::derived_from<TComponent, TReturnAsComponent>
    static std::pair<TReturnAsActor*, TReturnAsComponent*> findActorAndComponentByPath(const UWorld* world, const AActor* owner, const std::string& path)
    {
        bool get_component_by_path = true;
        return findActorAndComponent<TActor, TComponent, TReturnAsActor, TReturnAsComponent>(world, owner, path, get_component_by_path);
    }

    //
    // Find actors by name or tag or type and return an std::vector
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByName(const UWorld* world, const std::string& actor_name)
    {
        return findActorsByName<TReturnAsActor>(TActor::StaticClass(), world, actor_name);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTag(const UWorld* world, const std::string& tag)
    {
        return findActorsByTag<TReturnAsActor>(TActor::StaticClass(), world, tag);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTagsAny(const UWorld* world, const std::vector<std::string>& tags)
    {
        return findActorsByTagsAny<TReturnAsActor>(TActor::StaticClass(), world, tags);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByTagsAll(const UWorld* world, const std::vector<std::string>& tags)
    {
        return findActorsByTagsAll<TReturnAsActor>(TActor::StaticClass(), world, tags);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::vector<TReturnAsActor*> findActorsByType(const UWorld* world)
    {
        return findActorsByClass<TReturnAsActor>(TActor::StaticClass(), world);
    }

    // UClass* variants

    template <CActor TReturnAsActor = AActor>
    static std::vector<TReturnAsActor*> findActorsByName(UClass* uclass, const UWorld* world, const std::string& actor_name)
    {
        return Std::toVector<TReturnAsActor*>(
            findActorsByClass<TReturnAsActor>(uclass, world) |
            std::views::filter([](auto actor) { return hasStableName(actor); }) |
            std::views::filter([&actor_name](auto actor) { return getStableName(actor) == actor_name; }));
    }

    template <CActor TReturnAsActor = AActor>
    static std::vector<TReturnAsActor*> findActorsByTag(UClass* uclass, const UWorld* world, const std::string& tag)
    {
        return findActorsByTagsAny<TReturnAsActor>(uclass, world, {tag});
    }

    template <CActor TReturnAsActor = AActor>
    static std::vector<TReturnAsActor*> findActorsByTagsAny(UClass* uclass, const UWorld* world, const std::vector<std::string>& tags)
    {
        return Std::toVector<TReturnAsActor*>(
            findActorsByClass<TReturnAsActor>(uclass, world) |
            std::views::filter([&tags](auto actor) { return Std::any(Std::contains(Unreal::getTags(actor), tags)); }));
    }

    template <CActor TReturnAsActor = AActor>
    static std::vector<TReturnAsActor*> findActorsByTagsAll(UClass* uclass, const UWorld* world, const std::vector<std::string>& tags)
    {
        return Std::toVector<TReturnAsActor*>(
            findActorsByClass<TReturnAsActor>(uclass, world) |
            std::views::filter([&tags](auto actor) { return Std::all(Std::contains(Unreal::getTags(actor), tags)); }));
    }

    template <CActor TReturnAsActor = AActor>
    static std::vector<TReturnAsActor*> findActorsByClass(UClass* uclass, const UWorld* world)
    {
        SP_ASSERT(uclass);
        SP_ASSERT(uclass->IsChildOf(TReturnAsActor::StaticClass()));
        SP_ASSERT(world);
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
    static std::map<std::string, TReturnAsActor*> findActorsByNameAsMap(const UWorld* world, const std::string& actor_name)
    {
        return toMap(findActorsByName<TActor, TReturnAsActor>(world, actor_name));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAsMap(const UWorld* world, const std::string& tag)
    {
        return toMap(findActorsByTag<TActor, TReturnAsActor>(world, tag));
    }
    
    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagsAnyAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        return toMap(findActorsByTagsAny<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagsAllAsMap(const UWorld* world, const std::vector<std::string>& tags)
    {
        return toMap(findActorsByTagsAll<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTypeAsMap(const UWorld* world)
    {
        return toMap(findActorsByType<TActor, TReturnAsActor>(world));
    }

    // UClass* variants

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByNameAsMap(UClass* uclass, const UWorld* world, const std::string& actor_name)
    {
        return toMap(findActorsByName<TReturnAsActor>(uclass, world, actor_name));
    }

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAsMap(UClass* uclass, const UWorld* world, const std::string& tag)
    {
        return toMap(findActorsByTag<TReturnAsActor>(uclass, world, tag));
    }
    
    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagsAnyAsMap(UClass* uclass, const UWorld* world, const std::vector<std::string>& tags)
    {
        return toMap(findActorsByTagsAny<TReturnAsActor>(uclass, world, tags));
    }

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagsAllAsMap(UClass* uclass, const UWorld* world, const std::vector<std::string>& tags)
    {
        return toMap(findActorsByTagsAll<TReturnAsActor>(uclass, world, tags));
    }

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByClassAsMap(UClass* uclass, const UWorld* world)
    {
        return toMap(findActorsByClass<TReturnAsActor>(uclass, world));
    }

    //
    // Find actor by name or tag or type and return a pointer
    //

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByName(const UWorld* world, const std::string& actor_name)
    {
        return toItem(findActorsByName<TActor, TReturnAsActor>(world, actor_name));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTag(const UWorld* world, const std::string& tag)
    {
        return toItem(findActorsByTag<TActor, TReturnAsActor>(world, tag));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTagsAny(const UWorld* world, const std::vector<std::string>& tags)
    {
        return toItem(findActorsByTagsAny<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByTagsAll(const UWorld* world, const std::vector<std::string>& tags)
    {
        return toItem(findActorsByTagsAll<TActor, TReturnAsActor>(world, tags));
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static TReturnAsActor* findActorByType(const UWorld* world)
    {
        return toItem(findActorsByType<TActor, TReturnAsActor>(world));
    }

    // UClass* variants

    template <CActor TReturnAsActor = AActor>
    static TReturnAsActor* findActorByName(UClass* uclass, const UWorld* world, const std::string& actor_name)
    {
        return toItem(findActorsByName<TReturnAsActor>(uclass, world, actor_name));
    }

    template <CActor TReturnAsActor = AActor>
    static TReturnAsActor* findActorByTag(UClass* uclass, const UWorld* world, const std::string& tag)
    {
        return toItem(findActorsByTag<TReturnAsActor>(uclass, world, tag));
    }

    template <CActor TReturnAsActor = AActor>
    static TReturnAsActor* findActorByTagsAny(UClass* uclass, const UWorld* world, const std::vector<std::string>& tags)
    {
        return toItem(findActorsByTagsAny<TReturnAsActor>(uclass, world, tags));
    }

    template <CActor TReturnAsActor = AActor>
    static TReturnAsActor* findActorByTagsAll(UClass* uclass, const UWorld* world, const std::vector<std::string>& tags)
    {
        return toItem(findActorsByTagsAll<TReturnAsActor>(uclass, world, tags));
    }

    template <CActor TReturnAsActor = AActor>
    static TReturnAsActor* findActorByClass(UClass* uclass, const UWorld* world)
    {
        return toItem(findActorsByClass<TReturnAsActor>(uclass, world));
    }

    //
    // Get components by name or tag or type and return an std::vector
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByName(const AActor* actor, const std::string& component_name, bool include_from_child_actors = false)
    {
        return getComponentsByName<TReturnAsComponent>(TComponent::StaticClass(), actor, component_name, include_from_child_actors);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByPath(const AActor* actor, const std::string& component_path, bool include_from_child_actors = false)
    {
        return getComponentsByPath<TReturnAsComponent>(TComponent::StaticClass(), actor, component_path, include_from_child_actors);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTag(const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return getComponentsByTag<TReturnAsComponent>(TComponent::StaticClass(), actor, tag, include_from_child_actors);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagsAny(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return getComponentsByTagsAny<TReturnAsComponent>(TComponent::StaticClass(), actor, tags, include_from_child_actors);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagsAll(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return getComponentsByTagsAll<TReturnAsComponent>(TComponent::StaticClass(), actor, tags, include_from_child_actors);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::vector<TReturnAsComponent*> getComponentsByType(const AActor* actor, bool include_from_child_actors = false)
    {
        return getComponentsByClass<TReturnAsComponent>(TComponent::StaticClass(), actor, include_from_child_actors);
    }

    // UClass* variants

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::vector<TReturnAsComponent*> getComponentsByName(UClass* uclass, const AActor* actor, const std::string& component_name, bool include_from_child_actors = false)
    {
        return Std::toVector<TReturnAsComponent*>(
            getComponentsByClass<TReturnAsComponent>(uclass, actor, include_from_child_actors) |
            std::views::filter([](auto component) { return hasStableName(component); }) |
            std::views::filter([&component_name](auto component) { return getStableName(component) == component_name; }));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::vector<TReturnAsComponent*> getComponentsByPath(UClass* uclass, const AActor* actor, const std::string& component_path, bool include_from_child_actors = false)
    {
        // Given a component whose full name is DefaultSceneRoot.Parent.Child, we want to search for the
        // following names in this order: DefaultSceneRoot.Parent.Child, Parent.Child, Child

        std::vector<TReturnAsComponent*> components;

        std::vector<std::string> component_path_tokens = Std::tokenize(component_path, ".");
        std::vector<std::string> component_names;
        for (int i = 0; i < component_path_tokens.size(); i++) {
            std::string component_name = Std::join(Std::toVector<std::string>(component_path_tokens | std::views::drop(i)), ".");
            component_names.push_back(component_name);
        }

        for (auto& component_name : component_names) {
            std::vector<TReturnAsComponent*> current_components = getComponentsByName<TReturnAsComponent>(uclass, actor, component_name, include_from_child_actors);
            components.insert(components.end(), current_components.begin(), current_components.end()); // TODO: replace with components.append_range(children) in C++23
        }

        return components;
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTag(UClass* uclass, const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return getComponentsByTagsAny<TReturnAsComponent>(uclass, actor, {tag}, include_from_child_actors);
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagsAny(UClass* uclass, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return Std::toVector<TReturnAsComponent*>(
            getComponentsByClass<TReturnAsComponent>(uclass, actor, include_from_child_actors) |
            std::views::filter([&tags](auto component) { return Std::any(Std::contains(Unreal::getTags(component), tags)); }));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagsAll(UClass* uclass, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return Std::toVector<TReturnAsComponent*>(
            getComponentsByClass<TReturnAsComponent>(uclass, actor, include_from_child_actors) |
            std::views::filter([&tags](auto component) { return Std::all(Std::contains(Unreal::getTags(component), tags)); }));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::vector<TReturnAsComponent*> getComponentsByClass(UClass* uclass, const AActor* actor, bool include_from_child_actors = false)
    {
        SP_ASSERT(uclass);
        SP_ASSERT(uclass->IsChildOf(TReturnAsComponent::StaticClass()));
        SP_ASSERT(actor);
        TArray<UActorComponent*> components_tarray;
        actor->GetComponents(uclass, components_tarray, include_from_child_actors); // uclass can't be const because then we can't pass to GetComponents(...)
        std::vector<UActorComponent*> components = Unreal::toStdVector(components_tarray);
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
    static std::map<std::string, TReturnAsComponent*> getComponentsByNameAsMap(const AActor* actor, const std::string& component_name, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByName<TComponent, TReturnAsComponent>(actor, component_name, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByPathAsMap(const AActor* actor, const std::string& component_path, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByPath<TComponent, TReturnAsComponent>(actor, component_path, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAsMap(const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByTag<TComponent, TReturnAsComponent>(actor, tag, include_from_child_actors));
    }
    
    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagsAnyAsMap(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByTagsAny<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagsAllAsMap(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByTagsAll<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTypeAsMap(const AActor* actor, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByType<TComponent, TReturnAsComponent>(actor, include_from_child_actors));
    }

    // UClass* variants

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByNameAsMap(UClass* uclass, const AActor* actor, const std::string& component_name, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByName<TReturnAsComponent>(uclass, actor, component_name, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByPathAsMap(UClass* uclass, const AActor* actor, const std::string& component_path, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByPath<TReturnAsComponent>(uclass, actor, component_path, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAsMap(UClass* uclass, const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByTag<TReturnAsComponent>(uclass, actor, tag, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagsAnyAsMap(UClass* uclass, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByTagsAny<TReturnAsComponent>(uclass, actor, tags, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagsAllAsMap(UClass* uclass, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByTagsAll<TReturnAsComponent>(uclass, actor, tags, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByClassAsMap(UClass* uclass, const AActor* actor, bool include_from_child_actors = false)
    {
        return toMap(getComponentsByClass<TReturnAsComponent>(uclass, actor, include_from_child_actors));
    }

    //
    // Get component by name or tag or type and return a pointer
    //

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByName(const AActor* actor, const std::string& component_name, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByName<TComponent, TReturnAsComponent>(actor, component_name, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByPath(const AActor* actor, const std::string& component_path, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByPath<TComponent, TReturnAsComponent>(actor, component_path, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTag(const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByTag<TComponent, TReturnAsComponent>(actor, tag, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTagsAny(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByTagsAny<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByTagsAll(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByTagsAll<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors));
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static TReturnAsComponent* getComponentByType(const AActor* actor, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByType<TComponent, TReturnAsComponent>(actor, include_from_child_actors));
    }

    // UClass* variants

    template <CComponent TReturnAsComponent = UActorComponent>
    static TReturnAsComponent* getComponentByName(UClass* uclass, const AActor* actor, const std::string& component_name, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByName<TReturnAsComponent>(uclass, actor, component_name, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static TReturnAsComponent* getComponentByPath(UClass* uclass, const AActor* actor, const std::string& component_path, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByPath<TReturnAsComponent>(uclass, actor, component_path, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static TReturnAsComponent* getComponentByTag(UClass* uclass, const AActor* actor, const std::string& tag, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByTag<TReturnAsComponent>(uclass, actor, tag, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static TReturnAsComponent* getComponentByTagsAny(UClass* uclass, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByTagsAny<TReturnAsComponent>(uclass, actor, tags, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static TReturnAsComponent* getComponentByTagsAll(UClass* uclass, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByTagsAll<TReturnAsComponent>(uclass, actor, tags, include_from_child_actors));
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static TReturnAsComponent* getComponentByClass(UClass* uclass, const AActor* actor, bool include_from_child_actors = false)
    {
        return toItem(getComponentsByClass<TReturnAsComponent>(uclass, actor, include_from_child_actors));
    }

    //
    // Get children components by name or tag or type and return an std::vector
    //

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByName(const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true)
    {
        return getChildrenComponentsByName<TReturnAsSceneComponent>(TSceneComponent::StaticClass(), parent, child_component_name, include_all_descendants);
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTag(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return getChildrenComponentsByTag<TReturnAsSceneComponent>(TSceneComponent::StaticClass(), parent, tag, include_all_descendants);
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTagsAny(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return getChildrenComponentsByTagsAny<TReturnAsSceneComponent>(TSceneComponent::StaticClass(), parent, tags, include_all_descendants);
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTagsAll(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return getChildrenComponentsByTagsAll<TReturnAsSceneComponent>(TSceneComponent::StaticClass(), parent, tags, include_all_descendants);
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByType(const TParent* parent, bool include_all_descendants = true)
    {
        return getChildrenComponentsByClass<TReturnAsSceneComponent>(TSceneComponent::StaticClass(), parent, include_all_descendants);
    }

    // UClass* variants

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByName(UClass* uclass, const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true)
    {
        return Std::toVector<TReturnAsSceneComponent*>(
            getChildrenComponentsByClass<TReturnAsSceneComponent>(uclass, parent, include_all_descendants) |
            std::views::filter([](auto component) { return hasStableName(component); }) |
            std::views::filter([&child_component_name](auto component) { return getStableName(component) == child_component_name; }));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTag(UClass* uclass, const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return getChildrenComponentsByTagsAny<TReturnAsSceneComponent>(uclass, parent, {tag}, include_all_descendants);
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTagsAny(UClass* uclass, const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return Std::toVector<TReturnAsSceneComponent*>(
            getChildrenComponentsByClass<TReturnAsSceneComponent>(uclass, parent, include_all_descendants) |
            std::views::filter([&tags](auto component) { return Std::any(Std::contains(Unreal::getTags(component), tags)); }));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTagsAll(UClass* uclass, const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return Std::toVector<TReturnAsSceneComponent*>(
            getChildrenComponentsByClass<TReturnAsSceneComponent>(uclass, parent, include_all_descendants) |
            std::views::filter([&tags](auto component) { return Std::all(Std::contains(Unreal::getTags(component), tags)); }));
    }

    // specialization for AActor
    template <CSceneComponent TReturnAsSceneComponent = USceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByClass(UClass* uclass, const AActor* parent, bool include_all_descendants = true)
    {
        SP_ASSERT(uclass);
        SP_ASSERT(uclass->IsChildOf(TReturnAsSceneComponent::StaticClass()));
        SP_ASSERT(parent);
        USceneComponent* root = parent->GetRootComponent();
        std::vector<TReturnAsSceneComponent*> components;
        if (root) {
            SP_ASSERT(root->GetClass());
            if (root->GetClass()->IsChildOf(uclass)) {
                components.push_back(static_cast<TReturnAsSceneComponent*>(root));
            }
            if (include_all_descendants) {
                std::vector<TReturnAsSceneComponent*> children = getChildrenComponentsByClass<TReturnAsSceneComponent>(uclass, root, include_all_descendants);
                components.insert(components.end(), children.begin(), children.end()); // TODO: replace with components.append_range(children) in C++23
            }
        }
        return components;
    }

    // specialization for USceneComponent
    template <CSceneComponent TReturnAsSceneComponent = USceneComponent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByClass(UClass* uclass, const USceneComponent* parent, bool include_all_descendants = true)
    {
        SP_ASSERT(uclass);
        SP_ASSERT(uclass->IsChildOf(TReturnAsSceneComponent::StaticClass()));
        SP_ASSERT(parent);
        TArray<USceneComponent*> children_tarray;
        parent->GetChildrenComponents(include_all_descendants, children_tarray);
        std::vector<USceneComponent*> children = Unreal::toStdVector(children_tarray);
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

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByNameAsMap(const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByName<TSceneComponent, TReturnAsSceneComponent>(parent, child_component_name, include_all_descendants));
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagAsMap(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByTag<TSceneComponent, TReturnAsSceneComponent>(parent, tag, include_all_descendants));
    }
    
    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagsAnyAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByTagsAny<TSceneComponent, TReturnAsSceneComponent>(parent, tags, include_all_descendants));
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagsAllAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByTagsAll<TSceneComponent, TReturnAsSceneComponent>(parent, tags, include_all_descendants));
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTypeAsMap(const TParent* parent, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByType<TSceneComponent, TReturnAsSceneComponent>(parent, include_all_descendants));
    }

    // UClass* variants

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByNameAsMap(UClass* uclass, const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByName<TReturnAsSceneComponent>(uclass, parent, child_component_name, include_all_descendants));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagAsMap(UClass* uclass, const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByTag<TReturnAsSceneComponent>(uclass, parent, tag, include_all_descendants));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagsAnyAsMap(UClass* uclass, const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByTagsAny<TReturnAsSceneComponent>(uclass, parent, tags, include_all_descendants));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagsAllAsMap(UClass* uclass, const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByTagsAll<TReturnAsSceneComponent>(uclass, parent, tags, include_all_descendants));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByClassAsMap(UClass* uclass, const TParent* parent, bool include_all_descendants = true)
    {
        return toMap(getChildrenComponentsByClass<TReturnAsSceneComponent>(uclass, parent, include_all_descendants));
    }

    //
    // Get child component by name or tag or type and return a pointer
    //

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByName(const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByName<TSceneComponent, TReturnAsSceneComponent>(parent, child_component_name, include_all_descendants));
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByTag(const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByTag<TSceneComponent, TReturnAsSceneComponent>(parent, tag, include_all_descendants));
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByTagsAny(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByTagsAny<TSceneComponent, TReturnAsSceneComponent>(parent, tags, include_all_descendants));
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByTagsAll(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByTagsAll<TSceneComponent, TReturnAsSceneComponent>(parent, tags, include_all_descendants));
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* getChildComponentByType(const TParent* parent, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByType<TSceneComponent, TReturnAsSceneComponent>(parent, include_all_descendants));
    }

    // UClass* variants

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static TReturnAsSceneComponent* getChildComponentByName(UClass* uclass, const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByName<TReturnAsSceneComponent>(uclass, parent, child_component_name, include_all_descendants));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static TReturnAsSceneComponent* getChildComponentByTag(UClass* uclass, const TParent* parent, const std::string& tag, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByTag<TReturnAsSceneComponent>(uclass, parent, tag, include_all_descendants));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static TReturnAsSceneComponent* getChildComponentByTagsAny(UClass* uclass, const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByTagsAny<TReturnAsSceneComponent>(uclass, parent, tags, include_all_descendants));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static TReturnAsSceneComponent* getChildComponentByTagsAll(UClass* uclass, const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByTagsAll<TReturnAsSceneComponent>(uclass, parent, tags, include_all_descendants));
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static TReturnAsSceneComponent* getChildComponentByClass(UClass* uclass, const TParent* parent, bool include_all_descendants = true)
    {
        return toItem(getChildrenComponentsByClass<TReturnAsSceneComponent>(uclass, parent, include_all_descendants));
    }

    //
    // Helper functions to create components.
    //

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

    template <CNonSceneComponent TNonSceneComponent, CNonSceneComponent TReturnAsNonSceneComponent = TNonSceneComponent> requires
        std::derived_from<TNonSceneComponent, TReturnAsNonSceneComponent>
    static TReturnAsNonSceneComponent* createComponentInsideOwnerConstructor(AActor* owner, const std::string& component_name)
    {
        return Cast<TReturnAsNonSceneComponent>(createComponentInsideOwnerConstructorByClass(TNonSceneComponent::StaticClass(), owner, component_name)); // no RTTI available
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createSceneComponentInsideOwnerConstructor(AActor* owner, const std::string& scene_component_name)
    {
        return Cast<TReturnAsSceneComponent>(createSceneComponentInsideOwnerConstructorByClass(TSceneComponent::StaticClass(), owner, scene_component_name)); // no RTTI available
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createSceneComponentInsideOwnerConstructor(UObject* owner, USceneComponent* parent, const std::string& scene_component_name)
    {
        return Cast<TReturnAsSceneComponent>(createSceneComponentInsideOwnerConstructorByClass(TSceneComponent::StaticClass(), owner, parent, scene_component_name)); // no RTTI available
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createSceneComponentInsideOwnerConstructor(USceneComponent* owner, const std::string& scene_component_name)
    {
        return createSceneComponentInsideOwnerConstructor<TSceneComponent, TReturnAsSceneComponent>(owner, owner, scene_component_name);
    }

    static UActorComponent* createComponentInsideOwnerConstructorByClass(UClass* component_class, AActor* owner, const std::string& component_name)
    {
        SP_ASSERT(owner);
        UActorComponent* component = Cast<UActorComponent>(owner->CreateDefaultSubobject(Unreal::toFName(component_name), component_class, component_class, true, false)); // no RTTI available
        SP_ASSERT(component);
        return component;
    }

    static USceneComponent* createSceneComponentInsideOwnerConstructorByClass(UClass* scene_component_class, AActor* owner, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        USceneComponent* scene_component = Cast<USceneComponent>(owner->CreateDefaultSubobject(Unreal::toFName(scene_component_name), scene_component_class, scene_component_class, true, false)); // no RTTI available
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        return scene_component;
    }

    static USceneComponent* createSceneComponentInsideOwnerConstructorByClass(UClass* scene_component_class, UObject* owner, USceneComponent* parent, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        USceneComponent* scene_component = Cast<USceneComponent>(owner->CreateDefaultSubobject(Unreal::toFName(scene_component_name), scene_component_class, scene_component_class, true, false)); // no RTTI available
        SP_ASSERT(scene_component);
        scene_component->SetupAttachment(parent);
        return scene_component;
    }

    static USceneComponent* createSceneComponentInsideOwnerConstructorByClass(UClass* scene_component_class, USceneComponent* owner, const std::string& scene_component_name)
    {
        return createSceneComponentInsideOwnerConstructorByClass(scene_component_class, owner, owner, scene_component_name);
    }

    template <CNonSceneComponent TNonSceneComponent, CNonSceneComponent TReturnAsNonSceneComponent = TNonSceneComponent> requires
        std::derived_from<TNonSceneComponent, TReturnAsNonSceneComponent>
    static TReturnAsNonSceneComponent* createComponentOutsideOwnerConstructor(AActor* owner, const std::string& component_name)
    {
        return Cast<TReturnAsNonSceneComponent>(createComponentOutsideOwnerConstructorByClass(TNonSceneComponent::StaticClass(), owner, component_name)); // no RTTI available
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createSceneComponentOutsideOwnerConstructor(AActor* owner, const std::string& scene_component_name)
    {
        return Cast<TReturnAsSceneComponent>(createSceneComponentOutsideOwnerConstructorByClass(TSceneComponent::StaticClass(), owner, scene_component_name)); // no RTTI available
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createSceneComponentOutsideOwnerConstructor(UObject* owner, USceneComponent* parent, const std::string& scene_component_name)
    {
        return Cast<TReturnAsSceneComponent>(createSceneComponentOutsideOwnerConstructorByClass(TSceneComponent::StaticClass(), owner, parent, scene_component_name)); // no RTTI available
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createSceneComponentOutsideOwnerConstructor(USceneComponent* owner, const std::string& scene_component_name)
    {
        return createSceneComponentOutsideOwnerConstructor<TSceneComponent, TReturnAsSceneComponent>(owner, owner, scene_component_name);
    }

    static UActorComponent* createComponentOutsideOwnerConstructorByClass(UClass* component_class, AActor* owner, const std::string& component_name)
    {
        SP_ASSERT(owner);
        UActorComponent* component = NewObject<UActorComponent>(owner, component_class, Unreal::toFName(component_name));
        SP_ASSERT(component);
        component->RegisterComponent();
        return component;
    }

    static USceneComponent* createSceneComponentOutsideOwnerConstructorByClass(UClass* scene_component_class, AActor* owner, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        USceneComponent* scene_component = NewObject<USceneComponent>(owner, scene_component_class, Unreal::toFName(scene_component_name));
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        scene_component->RegisterComponent();
        return scene_component;
    }

    static USceneComponent* createSceneComponentOutsideOwnerConstructorByClass(UClass* scene_component_class, UObject* owner, USceneComponent* parent, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        USceneComponent* scene_component = NewObject<USceneComponent>(owner, scene_component_class, Unreal::toFName(scene_component_name));
        SP_ASSERT(scene_component);
        scene_component->SetupAttachment(parent);
        scene_component->RegisterComponent();
        return scene_component;
    }

    static USceneComponent* createSceneComponentOutsideOwnerConstructorByClass(UClass* scene_component_class, USceneComponent* owner, const std::string& scene_component_name)
    {
        return createSceneComponentOutsideOwnerConstructorByClass(scene_component_class, owner, owner, scene_component_name);
    }

    //
    // Helper function to destroy components.
    //

    static void destroyComponentOutsideOwnerConstructor(UActorComponent* component, bool promote_children = false)
    {
        component->DestroyComponent(promote_children);
    }

    //
    // Find objects
    //

    template <CObject TObject, CObject TReturnAsObject = TObject> requires
        std::derived_from<TObject, TReturnAsObject>
    static std::vector<TReturnAsObject*> findObjectsByType()
    {
        std::vector<TReturnAsObject*> objects;
        for (TObjectIterator<TObject> itr; itr; ++itr) {
            TReturnAsObject* object = *itr;
            objects.push_back(object);
        }
        return objects;
    }

    template <CObject TReturnAsObject = UObject>
    static std::vector<TReturnAsObject*> findObjectsByClass(const UClass* uclass)
    {
        SP_ASSERT(uclass);
        SP_ASSERT(uclass->IsChildOf(TReturnAsObject::StaticClass()));
        std::vector<TReturnAsObject*> objects;
        for (TObjectIterator<TReturnAsObject> itr; itr; ++itr) {
            TReturnAsObject* object = *itr;
            UClass* object_uclass = object->GetClass();
            SP_ASSERT(object_uclass);
            if (object_uclass->IsChildOf(uclass)) {
                objects.push_back(object);
            }
        }
        return objects;
    }

private:

    //
    // Helper functions for formatting container properties as strings in the same style as Unreal
    //

    static std::string getArrayPropertyValueAsFormattedString(const FProperty* property, const std::vector<std::string>& strings);
    static std::string getMapPropertyValueAsFormattedString(const FProperty* key_property, const std::vector<std::string>& key_strings, const FProperty* value_property, const std::vector<std::string>& value_strings);
    static std::string getQuoteStringForProperty(const FProperty* property);

    //
    // Helper functions for finding actors and getting components and getting properties
    //

    template <CActor TActor = AActor, CComponent TComponent = UActorComponent, CActor TReturnAsActor = TActor, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TActor, TReturnAsActor> && std::derived_from<TComponent, TReturnAsComponent>
    static std::pair<TReturnAsActor*, TReturnAsComponent*> findActorAndComponent(const UWorld* world, const AActor* owner, const std::string& path, bool get_component_by_path)
    {
        std::pair<TReturnAsActor*, TReturnAsComponent*> pair = std::make_pair(nullptr, nullptr);

        std::vector<std::string> path_tokens = Std::tokenize(path, ":");
        SP_ASSERT(path_tokens.size() == 1 || path_tokens.size() == 2);

        TReturnAsActor* actor = nullptr;
        TReturnAsComponent* component = nullptr;
        std::string component_path;

        if (path_tokens.size() == 1) {
            actor = const_cast<TReturnAsActor*>(Cast<TReturnAsActor>(owner)); // no RTTI available
            component_path = path_tokens.at(0);
        } else if (path_tokens.size() == 2) {
            actor = findActorByName<TActor, TReturnAsActor>(world, path_tokens.at(0));
            component_path = path_tokens.at(1);
        }

        if (get_component_by_path) {
            component = getComponentByPath<TComponent, TReturnAsComponent>(actor, component_path);
        } else {
            component = getComponentByName<TComponent, TReturnAsComponent>(actor, component_path);            
        }

        pair.first = actor;
        pair.second = component;

        return pair;
    }

    template <CStaticStruct TStaticStruct>
    static std::map<std::string, TStaticStruct*> toMap(const std::vector<TStaticStruct*>& objects)
    {
        // get names for all named objects
        std::vector<std::string> names = Std::toVector<std::string>(
            objects |
            std::views::transform([](auto object) { return getName(object); }));

        // count the number of occurances of each name
        std::map<std::string, uint64_t> counts = Std::count(names);

        // return a map of names to objects and filter by named object whose name occurs once in the input list
        return Std::toMap<std::string, TStaticStruct*>(
            objects |
            std::views::filter([&counts](auto object) { return counts.at(getName(object)) == 1; }) |
            std::views::transform([](auto object) { return std::make_pair(getName(object), object); }));
    }

    template <CNameObject TNameObject>
    static std::map<std::string, TNameObject*> toMap(const std::vector<TNameObject*>& objects)
    {
        // get stable names for all named objects
        std::vector<std::string> names = Std::toVector<std::string>(
            objects |
            std::views::transform([](auto object) { return Unreal::toStdString(object->GetName()); }));

        // count the number of occurances of each name
        std::map<std::string, uint64_t> counts = Std::count(names);

        // return a map of names to objects and filter by named object whose name occurs once in the input list
        return Std::toMap<std::string, TNameObject*>(
            objects |
            std::views::filter([&counts](auto object) { return counts.at(Unreal::toStdString(object->GetName())) == 1; }) |
            std::views::transform([](auto object) { return std::make_pair(Unreal::toStdString(object->GetName()), object); }));
    }

    template <CStableNameObject TStableNameObject>
    static std::map<std::string, TStableNameObject*> toMap(const std::vector<TStableNameObject*>& objects)
    {
        // get stable names for all named objects
        std::vector<std::string> names = Std::toVector<std::string>(
            objects |
            std::views::filter([](auto object) { return hasStableName(object); }) |
            std::views::transform([](auto object) { return getStableName(object); }));

        // count the number of occurances of each name
        std::map<std::string, uint64_t> counts = Std::count(names);

        // return a map of names to objects and filter by named object whose name occurs once in the input list
        return Std::toMap<std::string, TStableNameObject*>(
            objects |
            std::views::filter([](auto object) { return hasStableName(object); }) |
            std::views::filter([&counts](auto object) { return counts.at(getStableName(object)) == 1; }) |
            std::views::transform([](auto object) { return std::make_pair(getStableName(object), object); }));
    }

    template <typename TValue>
    static const TValue& toItem(const std::vector<TValue>& vector)
    {
        if (vector.size() == 0) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    ERROR: Input vector is empty.");
        } else if (vector.size() > 1) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    ERROR: Input vector has multiple entries: [", Std::join(Std::toVector<std::string>(vector | std::views::transform([](auto v) { return Std::toString(v); })), ", "), "]");
        }        
        SP_ASSERT(vector.size() == 1);
        return vector.at(0);
    }

    static std::string getName(UStruct* ustruct)
    {
        if (Cast<UFunction>(ustruct)) {
            return Unreal::toStdString(ustruct->GetName());
        } else {
            return Unreal::getTypeAsString(ustruct);
        }
    }
};
