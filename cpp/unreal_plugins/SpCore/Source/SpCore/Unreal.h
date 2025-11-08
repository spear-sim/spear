//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // int64_t, uint64_t

#include <concepts>    // std::constructible_from, std::derived_from, std::same_as
#include <map>
#include <memory>      // std::align
#include <ranges>      // std::views::filter, std::views::transform
#include <string>
#include <type_traits> // std::remove_pointer_t, std::underlying_type_t
#include <utility>     // std::make_pair, std::pair
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>
#include <Containers/UnrealString.h>        // FString::operator*
#include <Dom/JsonValue.h>
#include <Engine/Engine.h>                  // GEngine
#include <Engine/LocalPlayer.h>
#include <Engine/World.h>
#include <EngineUtils.h>                    // TActorIterator
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>                   // int32, int64, SPCORE_API, TCHAR
#include <Internationalization/Text.h>      // FText
#include <Subsystems/EngineSubsystem.h>
#include <Subsystems/Subsystem.h>
#include <Templates/Casts.h>
#include <UObject/Class.h>                  // EGetByNameFlags, EIncludeSuperFlag, UClass, UEnum, UScriptStruct, UStruct
#include <UObject/NameTypes.h>              // FName
#include <UObject/Object.h>                 // UObject
#include <UObject/ObjectMacros.h>           // EPropertyFlags
#include <UObject/ReflectedTypeAccessors.h> // StaticEnum
#include <UObject/Script.h>                 // EFunctionFlags
#include <UObject/UnrealType.h>             // EFieldIterationFlags, FProperty
#include <UObject/UObjectGlobals.h>         // NewObject
#include <UObject/UObjectIterator.h>

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"

//
// Concepts for Unreal objects
//

template <typename TObject>
concept CObject =
    std::derived_from<TObject, UObject>;

template <typename TObject>
concept CNonObject =
    !std::derived_from<TObject, UObject>;

// In the CStruct and CClass concepts below, there does not seem to be a clean way to encode the desired
// std::derived_from<...> relationships directly in the requires(...) { ... } statement of each concept. This
// is because, e.g., the type TStruct is implicitly passed as the first template parameter to std::derived_from<...>,
// but we actually need the type std::remove_pointer_t<TStruct> to be passed instead, in order for std::derived_from<...>
// to behave as expected. So we encode the std::derived_from<...> constraint outside the requires(...) { ... }
// statement in each concept, where we can manipulate TStruct and TClass more freely.

template <typename TEnum>
concept CEnum =
    requires () {
        { StaticEnum<TEnum>() } -> std::same_as<UEnum*>;
    };

template <typename TStruct>
concept CStruct =
    requires () {
        { TStruct::StaticStruct() }; // can't use std::same_as<UStruct*> because the type of the returned pointer might be derived from UStruct
    } &&
    std::derived_from<std::remove_pointer_t<decltype(TStruct::StaticStruct())>, UStruct>;

template <typename TClass>
concept CClass =
    CObject<TClass> &&
    requires () {
        { TClass::StaticClass() }; // can't use std::same_as<UClass*> because the type of the returned pointer might be derived from UClass
    } &&
    std::derived_from<std::remove_pointer_t<decltype(TClass::StaticClass())>, UClass>;

template <typename TInterface>
concept CInterface =
    CNonObject<TInterface> &&
    requires (TInterface interface) {
        { TInterface::UClassType::StaticClass() }; // can't use std::same_as<UClass*> because the type of the returned pointer might be derived from UClass
    } &&
    std::derived_from<std::remove_pointer_t<decltype(TInterface::UClassType::StaticClass())>, UClass>;

template <typename TComponent>
concept CComponent =
    CObject<TComponent> &&
    std::derived_from<TComponent, UActorComponent>;

template <typename TNonSceneComponent>
concept CNonSceneComponent =
    CComponent<TNonSceneComponent> &&
    !std::derived_from<TNonSceneComponent, USceneComponent>;

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

template <typename TNameObject>
concept CNameObject =
    !CStableNameObject<TNameObject> &&
    (requires (TNameObject name_object) {
        { name_object.GetName() } -> std::same_as<FName>;
    } ||
    requires (TNameObject name_object) {
        { name_object.GetName() } -> std::same_as<FString>;
    });

template <typename TParent>
concept CParent =
    CActor<TParent> || CSceneComponent<TParent>;

template <typename TPropertyContainer>
concept CPropertyContainer =
    std::derived_from<TPropertyContainer, UStruct> || std::derived_from<TPropertyContainer, UFunction>;

template <typename TSubsystem>
concept CSubsystem =
    CClass<TSubsystem> &&
    std::derived_from<TSubsystem, USubsystem>;

template <typename TEngineSubsystem>
concept CEngineSubsystem =
    CSubsystem<TEngineSubsystem> &&
    std::derived_from<TEngineSubsystem, UEngineSubsystem>;

template <typename TSubsystemProvider>
concept CSubsystemProvider =
    CClass<TSubsystemProvider> &&
    requires (TSubsystemProvider subsystem_provider, UClass* uclass) {
        { subsystem_provider.template GetSubsystem<USubsystem>() } -> std::same_as<USubsystem*>;
        { subsystem_provider.GetSubsystemBase(nullptr) }; // can't use std::same_as<USubsystem*> because the type of the returned pointer might be derived from USubsystem
    } &&
    std::derived_from<std::remove_pointer_t<decltype(TSubsystemProvider().GetSubsystemBase(nullptr))>, USubsystem>;

//
// SpPropertyDesc encapsulates the data needed to get and set a UPROPERTY on a specific UObject.
//

struct SpPropertyDesc
{
    FProperty* property_ = nullptr;
    void* value_ptr_ = nullptr;
};

//
// General-purpose functions for working with Unreal objects.
//

class SPCORE_API Unreal
{
public:
    Unreal() = delete;
    ~Unreal() = delete;

    //
    // Helper function for static structs and classes
    //

    template <CStruct TStruct>
    static UStruct* getStaticStruct()
    {
        return TStruct::StaticStruct();
    }

    template <CClass TClass>
    static UStruct* getStaticStruct()
    {
        return TClass::StaticClass();
    }

    template <CInterface TInterface>
    static UStruct* getStaticStruct()
    {
        return TInterface::UClassType::StaticClass();
    }

    static std::vector<UScriptStruct*> findStaticStructs()
    {
        return Unreal::findObjects<UScriptStruct>();
    }

    static std::map<std::string, UScriptStruct*> findStaticStructsAsMap(bool use_cpp_type_as_key = true)
    {
        std::map<std::string, UScriptStruct*> script_structs_map;
        std::vector<UScriptStruct*> script_structs = findStaticStructs();
        if (use_cpp_type_as_key) {
            script_structs_map = Std::toMap<std::string, UScriptStruct*>(
                script_structs |
                std::views::transform([](auto script_struct) { return std::make_pair(Unreal::getCppTypeAsString(script_struct), script_struct); }));
        } else {
            script_structs_map = toMap(script_structs);
        }
        return script_structs_map;
    }

    template <CClass TClass>
    static UClass* getStaticClass()
    {
        return TClass::StaticClass();
    }

    template <CInterface TInterface>
    static UClass* getStaticClass()
    {
        return TInterface::UClassType::StaticClass();
    }

    static std::vector<UClass*> getDerivedClasses(UClass* uclass, bool recursive)
    {
        TArray<UClass*> derived_uclasses;
        GetDerivedClasses(uclass, derived_uclasses, recursive);
        return Unreal::toStdVector(derived_uclasses);
    }

    static std::map<std::string, UClass*> getDerivedClassesAsMap(UClass* uclass, bool recursive = true, bool use_cpp_type_as_key = true)
    {
        std::map<std::string, UClass*> uclasses_map;
        std::vector<UClass*> uclasses = Unreal::getDerivedClasses(uclass, recursive);
        if (use_cpp_type_as_key) {
            uclasses_map = Std::toMap<std::string, UClass*>(
                uclasses |
                std::views::transform([](auto uclass) { return std::make_pair(Unreal::getCppTypeAsString(uclass), uclass); }));
        } else {
            uclasses_map = toMap(uclasses);
        }
        return uclasses_map;
    }

    template <typename TStructOrClassOrInterface> requires
         CStruct<TStructOrClassOrInterface> || CClass<TStructOrClassOrInterface> || CInterface<TStructOrClassOrInterface>
    static std::string getCppTypeAsString()
    {
        UStruct* ustruct = getStaticStruct<TStructOrClassOrInterface>();
        return getCppTypeAsString(ustruct);
    }

    static std::string getCppTypeAsString(UStruct* ustruct)
    {
        return Unreal::toStdString(ustruct->GetPrefixCPP()) + Unreal::toStdString(ustruct->GetName());
    }

    static std::string getCppTypeAsString(FProperty* property)
    {
        return Unreal::toStdString(property->GetCPPType());
    }

    //
    // Helper function for finding objects
    //

    template <CObject TObject>
    static std::vector<TObject*> findObjects()
    {
        std::vector<TObject*> objects;
        for (TObjectIterator<TObject> itr; itr; ++itr) {
            TObject* object = *itr;
            objects.push_back(object);
        }
        return objects;
    }

    //
    // Find special struct by name. For this function to behave as expected, USpSpecialStructs must have a
    // UPROPERTY defined on it named TypeName_ of type TypeName.
    //

    static UStruct* findSpecialStructByName(const std::string& struct_name);

    //
    // Get property metadata
    //

    template <CPropertyContainer TPropertyContainer>
    static std::vector<FProperty*> findProperties(const TPropertyContainer* property_container, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default)
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
            findProperties(property_container, field_iteration_flags) |
            std::views::filter([property_flags](auto property) { return property->HasAnyPropertyFlags(property_flags); }));
    }

    template <CPropertyContainer TPropertyContainer>
    static std::vector<FProperty*> findPropertiesByFlagsAll(const TPropertyContainer* property_container, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default)
    {
        return Std::toVector<FProperty*>(
            findProperties(property_container, field_iteration_flags) |
            std::views::filter([property_flags](auto property) { return property->HasAllPropertyFlags(property_flags); }));
    }

    template <CPropertyContainer TPropertyContainer>
    static std::map<std::string, FProperty*> findPropertiesAsMap(const TPropertyContainer* property_container, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default)
    {
        return toMap(findProperties(property_container, field_iteration_flags));
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
    // Get and set object properties, uobject can't be const because we cast it to void*
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

    static std::string getPropertyValueAsString(const SpPropertyDesc& property_desc);
    static void setPropertyValueFromString(const SpPropertyDesc& property_desc, const std::string& string);
    static void setPropertyValueFromJsonValue(const SpPropertyDesc& property_desc, TSharedPtr<FJsonValue> json_value);

    //
    // Get functions, find function by name, call function. Note that uobject can't be const because we call
    // uobject->ProcessEvent(...) which is non-const; and ufunction can't be const because we pass it to uobject->ProcessEvent(...)
    // which expects non-const.
    //

    static std::vector<UFunction*> findFunctions(const UClass* uclass, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::vector<UFunction*> findFunctionsByFlagsAny(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::vector<UFunction*> findFunctionsByFlagsAll(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::map<std::string, UFunction*> findFunctionsAsMap(const UClass* uclass, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::map<std::string, UFunction*> findFunctionsByFlagsAnyAsMap(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::map<std::string, UFunction*> findFunctionsByFlagsAllAsMap(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static UFunction* findFunctionByName(const UClass* uclass, const std::string& function_name, EIncludeSuperFlag::Type include_super_flag = EIncludeSuperFlag::IncludeSuper);
    static std::map<std::string, std::string> callFunction(const UWorld* world, UObject* uobject, UFunction* ufunction, const std::map<std::string, std::string>& args = {}, const std::string& world_context = "WorldContextObject");

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
        UActorComponent* component = Cast<UActorComponent>(owner->CreateDefaultSubobject(toFName(component_name), component_class, component_class, true, false)); // no RTTI available
        SP_ASSERT(component);
        return component;
    }

    static USceneComponent* createSceneComponentInsideOwnerConstructorByClass(UClass* scene_component_class, AActor* owner, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        USceneComponent* scene_component = Cast<USceneComponent>(owner->CreateDefaultSubobject(toFName(scene_component_name), scene_component_class, scene_component_class, true, false)); // no RTTI available
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        return scene_component;
    }

    static USceneComponent* createSceneComponentInsideOwnerConstructorByClass(UClass* scene_component_class, UObject* owner, USceneComponent* parent, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        USceneComponent* scene_component = Cast<USceneComponent>(owner->CreateDefaultSubobject(toFName(scene_component_name), scene_component_class, scene_component_class, true, false)); // no RTTI available
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
        UActorComponent* component = NewObject<UActorComponent>(owner, component_class, toFName(component_name));
        SP_ASSERT(component);
        component->RegisterComponent();
        return component;
    }

    static USceneComponent* createSceneComponentOutsideOwnerConstructorByClass(UClass* scene_component_class, AActor* owner, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        USceneComponent* scene_component = NewObject<USceneComponent>(owner, scene_component_class, toFName(scene_component_name));
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        scene_component->RegisterComponent();
        return scene_component;
    }

    static USceneComponent* createSceneComponentOutsideOwnerConstructorByClass(UClass* scene_component_class, UObject* owner, USceneComponent* parent, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        USceneComponent* scene_component = NewObject<USceneComponent>(owner, scene_component_class, toFName(scene_component_name));
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

    template <CParent TParent>
    static std::vector<USceneComponent*> getChildrenComponents(const TParent* parent, bool include_all_descendants = true)
    {
        return getChildrenComponentsByType<USceneComponent>(parent, include_all_descendants);
    }

    template <CParent TParent>
    static std::map<std::string, USceneComponent*> getChildrenComponentsAsMap(const TParent* parent, bool include_all_descendants = true)
    {
        return toMap<USceneComponent>(getChildrenComponents(parent, include_all_descendants));
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
    static std::vector<TReturnAsActor*> findActorsByName(const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsActor*> actors;
        std::map<std::string, TReturnAsActor*> all_actors_map = toMap<TReturnAsActor>(findActorsByType<TActor>(world));

        if (Config::isInitialized() && Config::get<bool>("SP_CORE.PRINT_FIND_ACTOR_AND_GET_COMPONENT_DEBUG_INFO")) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    Query names: ", Std::join(actor_names, ", "));
            SP_LOG("    Actor names: ", Std::join(Std::keys(all_actors_map), ", "));
        }

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

        if (Config::isInitialized() && Config::get<bool>("SP_CORE.PRINT_FIND_ACTOR_AND_GET_COMPONENT_DEBUG_INFO")) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    Query names:     ", Std::join(component_names, ", "));
            SP_LOG("    Component names: ", Std::join(Std::keys(all_components_map), ", "));
        }

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
    static std::vector<TReturnAsComponent*> getComponentsByPath(
        const AActor* actor, const std::vector<std::string>& component_paths, bool include_from_child_actors = false, bool return_null_if_not_found = true)
    {
        std::vector<TReturnAsComponent*> components;
        std::map<std::string, TReturnAsComponent*> all_components_map = toMap<TReturnAsComponent>(getComponentsByType<TComponent>(actor, include_from_child_actors));

        if (Config::isInitialized() && Config::get<bool>("SP_CORE.PRINT_FIND_ACTOR_AND_GET_COMPONENT_DEBUG_INFO")) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    Query paths:     ", Std::join(component_paths, ", "));
            SP_LOG("    Component paths: ", Std::join(Std::keys(all_components_map), ", "));
        }

        for (auto& component_path : component_paths) {
            std::vector<std::string> component_path_tokens = Std::tokenize(component_path, ".");
            std::string component_name = component_path_tokens.at(component_path_tokens.size() - 1);
            if (Std::containsKey(all_components_map, component_path)) {
                components.push_back(all_components_map.at(component_path));
            } else if (Std::containsKey(all_components_map, component_name)) {
                components.push_back(all_components_map.at(component_name));
            } else if (return_null_if_not_found) {
                components.push_back(nullptr);
            }
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
    static std::map<std::string, TReturnAsComponent*> getComponentsByPathAsMap(
        const AActor* actor, const std::vector<std::string>& component_paths, bool include_from_child_actors = false, bool return_null_if_not_found = true)
    {
        std::map<std::string, TReturnAsComponent*> component_map;
        std::vector<std::string> unique_component_paths = Std::unique(component_paths);

        if (return_null_if_not_found) {
            component_map = Std::zip(unique_component_paths, getComponentsByPath<TComponent, TReturnAsComponent>(actor, unique_component_paths, return_null_if_not_found));
        } else {
            std::vector<TComponent*> components = getComponentsByPath<TComponent>(actor, unique_component_paths, return_null_if_not_found);
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
    static TReturnAsComponent* getComponentByPath(const AActor* actor, const std::string& component_path, bool include_from_child_actors = false)
    {
        bool return_null_if_not_found = false;
        return getItem(getComponentsByPath<TComponent, TReturnAsComponent>(actor, {component_path}, include_from_child_actors, return_null_if_not_found));
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

        if (Config::isInitialized() && Config::get<bool>("SP_CORE.PRINT_FIND_ACTOR_AND_GET_COMPONENT_QUERY_INFO")) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    Query names:              ", Std::join(children_component_names, ", "));
            SP_LOG("    Children component names: ", Std::join(Std::keys(all_components_map), ", "));
        }

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
    // Helper functions for getting subsystems, world can't be const because we may need to return it
    // directly in some specializations
    //

    template <CSubsystemProvider TSubsystemProvider>
    static TSubsystemProvider* getSubsystemProvider(UWorld* world)
    {
        SP_ASSERT(false);
        return nullptr;
    }

    template <>
    ULocalPlayer* getSubsystemProvider<ULocalPlayer>(UWorld* world)
    {
        SP_ASSERT(world);
        APlayerController* player_controller = world->GetFirstPlayerController();
        SP_ASSERT(player_controller);
        ULocalPlayer* local_player = player_controller->GetLocalPlayer();
        SP_ASSERT(local_player);
        return local_player;
    }

    //
    // Get engine subsystem, uclass can't be const because we need to pass it to GetEngineSubsystemBase(...)
    //

    template <CEngineSubsystem TEngineSubsystem>
    static TEngineSubsystem* getEngineSubsystemByType()
    {
        SP_ASSERT(GEngine);
        return GEngine->GetEngineSubsystem<TEngineSubsystem>();
    }

    static UEngineSubsystem* getEngineSubsystemByClass(UClass* uclass)
    {
        SP_ASSERT(GEngine);
        return GEngine->GetEngineSubsystemBase(uclass);
    }

    //
    // Get subsystem, world can't be const because we need to return it directly in some specializations, and
    // uclass can't be const because we need to pass it to GetSubsystemBase(...)
    //

    template <CSubsystem TSubsystem, CSubsystemProvider TSubsystemProvider>
    static TSubsystem* getSubsystemByType(UWorld* world)
    {
        return getSubsystemProvider<TSubsystemProvider>(world)->template GetSubsystem<TSubsystem>();
    }

    template <CSubsystemProvider TSubsystemProvider>
    static USubsystem* getSubsystemByClass(UWorld* world, UClass* uclass)
    {
        return getSubsystemProvider<TSubsystemProvider>(world)->GetSubsystemBase(uclass);
    }

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
        return getEnumValueAs<TEnum>(uenum->GetValueByName(toFName(string), EGetByNameFlags::ErrorIfNotFound | EGetByNameFlags::CaseSensitive));
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
        std::string str = toStdString(uenum->GetNameStringByValue(getEnumValue(src)));
        SP_ASSERT(str != "");
        return str;
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
        for (auto& s : src) {
            dest.push_back(s);
        }
        return dest;
    }

    template <typename TDestValue, typename TSrcValue>
    static std::vector<TDestValue> toStdVectorOf(const TArray<TSrcValue>& src)
    {
        std::vector<TDestValue> dest;
        for (auto& s : src) {
            dest.push_back(s);
        }
        return dest;
    }

    template <typename TValue>
    static TArray<TValue> toTArray(const std::vector<TValue>& src)
    {
        TArray<TValue> dest;
        for (auto& s : src) {
            dest.Add(s);
        }
        return dest;
    }

    template <typename TDestValue, typename TValue>
    static TArray<TDestValue> toTArrayOf(const std::vector<TValue>& src)
    {
        TArray<TDestValue> dest;
        for (auto& s : src) {
            dest.Add(s);
        }
        return dest;
    }

    template <typename TValue>
    static TValue* updateArrayDataPtr(TArray<TValue>& array, void* data_ptr, uint64_t num_bytes) {

        SP_ASSERT(num_bytes % sizeof(TValue) == 0);
        uint64_t num_elements = num_bytes / sizeof(TValue);

        //
        // We enforce the constraint that array's existing data region must be at least as big as the data_ptr
        // region, because we want to guarantee that array will not resize itself if the user adds elements
        // that would fit in the data_ptr region. However, we don't test for exact equality between the size
        // of the array's data region and the data_ptr data region. This is because calling array.Reserve(num_elements)
        // is allowed to internally allocate more than num_elements worth of space.
        //
        // After calling updateArrayDataPtr(...), the user must be careful not to add more than num_elements
        // elements to array. If the user adds too many elements, array will write past the end of the data_ptr
        // region, and eventually trigger a resize operation, in which case it will no longer be backed by
        // the user's data_ptr region at all. Both of these outcomes is undesirable, and therefore the user
        // must be careful not to add more than num_elements to array.
        //

        SP_ASSERT(array.Max() >= 0);
        SP_ASSERT(num_elements <= static_cast<uint64_t>(array.Max()));
        SP_ASSERT(num_bytes <= array.GetAllocatedSize());
        SP_ASSERT(Std::isPtrSufficientlyAlignedFor<TValue>(data_ptr));

        // Get pointer to array object, interpret as a pointer-to-TValue*.
        TValue** array_ptr = reinterpret_cast<TValue**>(&array);
        SP_ASSERT(array_ptr);

        TValue* array_data_ptr = array.GetData();

        // Check that the pointer to the array object, when interpreted as a pointer-to-TValue*, does indeed
        // point to the array's underlying data.
        SP_ASSERT(*array_ptr == array.GetData());

        // Update the array's underlying data pointer.
        *array_ptr = static_cast<TValue*>(data_ptr);
        SP_ASSERT(data_ptr == array.GetData());

        return array_data_ptr;
    };

    //
    // String functions
    //

    static std::string toStdString(const FName& str);
    static std::string toStdString(const FString& str);
    static std::string toStdString(const FText& str);
    static std::string toStdString(const TCHAR* str);

    static FName toFName(const std::string& str);
    static FString toFString(const std::string& str);
    static FText toFText(const std::string& str);

    struct TCharPtr
    {
        TCharPtr() = delete;
        TCharPtr(const std::string& str) { str_ = toFString(str); }
        TCharPtr(const FString& str) { str_ = str; }
        TCharPtr(const char* str) { str_ = str; }

        operator const TCHAR* () const { return *str_; }

        FString str_;
    };

    template <typename TStr> requires
        std::constructible_from<TCharPtr, TStr>
    static Unreal::TCharPtr toTCharPtr(TStr&& str)
    {
        return TCharPtr(std::forward<TStr>(str));
    }

private:

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
            actor = Unreal::findActorByName<TActor, TReturnAsActor>(world, path_tokens.at(0));
            component_path = path_tokens.at(1);
        }

        if (get_component_by_path) {
            component = Unreal::getComponentByPath<TComponent, TReturnAsComponent>(actor, component_path);
        } else {
            component = Unreal::getComponentByName<TComponent, TReturnAsComponent>(actor, component_path);            
        }

        pair.first = actor;
        pair.second = component;

        return pair;
    }

    template <typename TReturnAsStableNameObject, typename TStableNameObject> requires
        CStableNameObject<TStableNameObject> && std::derived_from<TStableNameObject, TReturnAsStableNameObject>
    static std::map<std::string, TReturnAsStableNameObject*> toMap(const std::vector<TStableNameObject*>& objects)
    {
        return Std::toMap<std::string, TReturnAsStableNameObject*>(
            objects |
            std::views::filter([](auto object) { return hasStableName(object); }) |
            std::views::transform([](auto object) { return std::make_pair(getStableName(object), static_cast<TReturnAsStableNameObject*>(object)); }));
    }

    template <typename TNameObject> requires
        CNameObject<TNameObject>
    static std::map<std::string, TNameObject*> toMap(const std::vector<TNameObject*>& name_objects)
    {
        return Std::toMap<std::string, TNameObject*>(
            name_objects |
            std::views::transform([](auto name_object) { return std::make_pair(Unreal::toStdString(name_object->GetName()), name_object); }));
    }

    template <typename TValue>
    static const TValue& getItem(const std::vector<TValue>& vector)
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
