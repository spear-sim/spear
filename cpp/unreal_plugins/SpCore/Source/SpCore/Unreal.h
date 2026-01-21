//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t

#include <concepts>    // std::constructible_from, std::derived_from, std::same_as
#include <map>
#include <string>
#include <ranges>      // std::views::transform
#include <type_traits> // std::remove_pointer_t, std::underlying_type_t
#include <utility>     // std::forward
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>
#include <Containers/Map.h>
#include <Containers/UnrealString.h>        // FString::operator*
#include <Engine/Engine.h>                  // GEngine
#include <Engine/LocalPlayer.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <GameFramework/PlayerController.h>
#include <HAL/Platform.h>                   // int32, int64, SPCORE_API, TCHAR
#include <Internationalization/Text.h>      // FText
#include <StructUtils/UserDefinedStruct.h>
#include <Subsystems/EngineSubsystem.h>
#include <Subsystems/Subsystem.h>
#include <UObject/Class.h>                  // EGetByNameFlags, TBaseStructure, UClass, UEnum, UFunction, UScriptStruct, UStruct
#include <UObject/NameTypes.h>              // FName
#include <UObject/Object.h>                 // UObject
#include <UObject/ReflectedTypeAccessors.h> // StaticEnum
#include <UObject/UnrealType.h>             // FProperty

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

//
// Concepts for Unreal objects
//

// In the CStruct and CClass concepts below, there does not seem to be a clean way to encode the desired
// std::derived_from<...> relationships directly in the requires(...) { ... } statement of each concept. This
// is because, e.g., the type TStruct is implicitly passed as the first template parameter to std::derived_from<...>,
// but we actually need the type std::remove_pointer_t<TStruct> to be passed instead, in order for std::derived_from<...>
// to behave as expected. So we encode the std::derived_from<...> constraint outside the requires(...) { ... }
// statement in each concept, where we can manipulate TStruct and TClass more freely.

template <typename TStaticStruct>
concept CStaticStruct =
    std::derived_from<TStaticStruct, UStruct> &&
    !std::derived_from<TStaticStruct, UFunction>;

template <typename TObject>
concept CNonObject =
    !std::derived_from<TObject, UObject>;

template <typename TObject>
concept CObject =
    std::derived_from<TObject, UObject>;

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

template <typename TSpecialStruct>
concept CSpecialStruct =
        !(CStruct<TSpecialStruct> || CClass<TSpecialStruct> || CInterface<TSpecialStruct>);

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
    !CStaticStruct<TNameObject> &&
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
// General-purpose functions for working with Unreal objects.
//

class SPCORE_API Unreal
{
public:
    Unreal() = delete;
    ~Unreal() = delete;

    //
    // Helper functions for static structs
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

    template <CSpecialStruct TSpecialStruct>
    static UStruct* getStaticStruct()
    {
        return TBaseStructure<TSpecialStruct>::Get();
    }

    //
    // Helper function for static classes
    //

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

    //
    // Helper functions for getting types as strings
    //

    static std::string getTypeAsString(UStruct* ustruct)
    {
        SP_ASSERT(ustruct);
        std::string str;
        if (ustruct->IsA(UUserDefinedStruct::StaticClass())) {
            str = getBlueprintTypeAsString(ustruct);
        } else if (ustruct->IsA(UScriptStruct::StaticClass())) {
            str = getCppTypeAsString(ustruct);
        } else if (ustruct->IsA(UClass::StaticClass())) {
            UClass* uclass = static_cast<UClass*>(ustruct);
            if (uclass->HasAnyClassFlags(EClassFlags::CLASS_Native)) {
                str = getCppTypeAsString(ustruct);
            } else {
                str = getBlueprintTypeAsString(ustruct);
            }
        } else {
            SP_ASSERT(false);
        }
        return str;
    }

    template <typename TStructOrClassOrInterface> requires
         (CSpecialStruct<TStructOrClassOrInterface> || CStruct<TStructOrClassOrInterface> || CClass<TStructOrClassOrInterface> || CInterface<TStructOrClassOrInterface>)
    static std::string getCppTypeAsString()
    {
        UStruct* ustruct = getStaticStruct<TStructOrClassOrInterface>();
        return getCppTypeAsString(ustruct);
    }

    static std::string getCppTypeAsString(UStruct* ustruct)
    {
        SP_ASSERT(ustruct);
        if (ustruct->IsA(UUserDefinedStruct::StaticClass())) {
            SP_ASSERT(false);
        } else if (ustruct->IsA(UScriptStruct::StaticClass())) {
            // pass
        } else if (ustruct->IsA(UClass::StaticClass())) {
            UClass* uclass = static_cast<UClass*>(ustruct);
            SP_ASSERT(uclass->HasAnyClassFlags(EClassFlags::CLASS_Native));
        } else {
            SP_ASSERT(false);
        }
        std::string str = toStdString(ustruct->GetPrefixCPP()) + toStdString(ustruct->GetName());
        SP_ASSERT(str != "");
        return str;
    }

    static std::string getCppTypeAsString(FProperty* property)
    {
        std::string str = toStdString(property->GetCPPType());
        SP_ASSERT(str != "");
        return str;
    }

    static std::string getBlueprintTypeAsString(UStruct* ustruct)
    {
        SP_ASSERT(ustruct);
        if (ustruct->IsA(UUserDefinedStruct::StaticClass())) {
            // pass
        } else if (ustruct->IsA(UScriptStruct::StaticClass())) {
            SP_ASSERT(false);
        } else if (ustruct->IsA(UClass::StaticClass())) {
            UClass* uclass = static_cast<UClass*>(ustruct);
            SP_ASSERT(!uclass->HasAnyClassFlags(EClassFlags::CLASS_Native)); // EClassFlags::CLASS_CompiledFromBlueprint is not always set in this case
        } else {
            SP_ASSERT(false);
        }
        std::string str = toStdString(ustruct->GetName());
        SP_ASSERT(str != "");
        return str;
    }

    //
    // Helper functions for getting subsystems, world can't be const because we may need to return it
    // directly in some specializations
    //

    template <CSubsystemProvider TSubsystemProvider>
    static TSubsystemProvider* getSubsystemProvider(UWorld* world) { SP_ASSERT(false); return nullptr; }

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

    template <>
    UWorld* getSubsystemProvider<UWorld>(UWorld* world)
    {
        SP_ASSERT(world);
        return world;
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
    // Get object tags
    //

    static std::vector<std::string> getTags(const AActor* actor)
    {
        SP_ASSERT(actor);
        return Std::toVector<std::string>(toStdVector(actor->Tags) | std::views::transform([](const auto& tag) { return toStdString(tag); }));
    }

    static std::vector<std::string> getTags(const UActorComponent* component)
    {
        SP_ASSERT(component);
        return Std::toVector<std::string>(toStdVector(component->ComponentTags) | std::views::transform([](const auto& tag) { return toStdString(tag); }));
    }

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

    template <typename TKey, typename TValue>
    static std::map<TKey, TValue> toStdMap(const TMap<TKey, TValue>& src)
    {
        std::map<TKey, TValue> dest;
        for (auto& [k, v] : src) {
            Std::insert(dest, k, v);
        }
        return dest;
    }

    template <typename TKey, typename TValue>
    static TMap<TKey, TValue> toTMap(const std::map<TKey, TValue>& src)
    {
        TMap<TKey, TValue> dest;
        for (auto& [k, v] : src) {
            dest.Add(k, v);
        }
        return dest;
    }

    //
    // String functions
    //

    static std::string toStdString(const FName& str)
    {
        return toStdString(str.ToString()); // str.ToString() converts FName to FString
    }

    static std::string toStdString(const FString& str)
    {
        return std::string(TCHAR_TO_UTF8(*str)); // the * operator for FString returns a pointer to the underlying TCHAR array
    }

    static std::string toStdString(const FText& str)
    {
        return toStdString(str.ToString()); // str.ToString() converts FText to FString
    }

    static std::string toStdString(const TCHAR* str)
    {
        SP_ASSERT(str);
        return std::string(TCHAR_TO_UTF8(str));
    }

    static FName toFName(const std::string& str)
    {
        return FName(str.c_str());
    }

    static FString toFString(const std::string& str)
    {
        return FString(UTF8_TO_TCHAR(str.c_str()));
    }

    static FText toFText(const std::string& str)
    {
        return FText::FromString(toFString(str));
    }

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
};
