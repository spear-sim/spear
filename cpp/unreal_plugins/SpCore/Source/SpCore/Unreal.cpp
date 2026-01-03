//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/Unreal.h"

#include <stdint.h> // uint8_t

#include <map>
#include <ranges>  // std::views::transform
#include <string>
#include <utility> // std::move
#include <vector>

#include <Components/ActorComponent.h>
#include <Containers/Array.h>
#include <Containers/StringConv.h>     // TCHAR_TO_UTF8, UTF8_TO_TCHAR
#include <Containers/UnrealString.h>   // FString::operator*
#include <Dom/JsonObject.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>              // TCHAR, uint16
#include <Internationalization/Text.h> // FText
#include <JsonObjectConverter.h>
#include <Serialization/JsonReader.h>
#include <Serialization/JsonSerializer.h>
#include <Templates/SharedPointer.h>   // TSharedPtr, TSharedRef
#include <UObject/NameTypes.h>         // FName
#include <UObject/Class.h>             // UFunction
#include <UObject/Object.h>            // UObject
#include <UObject/ObjectMacros.h>      // EPropertyFlags
#include <UObject/UnrealType.h>        // FProperty, TFieldIterator

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

class UStruct;
class UWorld;

//
// Get object tags
//

std::vector<std::string> Unreal::getTags(const AActor* actor)
{
    SP_ASSERT(actor);
    return Std::toVector<std::string>(toStdVector(actor->Tags) | std::views::transform([](const auto& tag) { return toStdString(tag); }));
}

std::vector<std::string> Unreal::getTags(const UActorComponent* component)
{
    SP_ASSERT(component);
    return Std::toVector<std::string>(toStdVector(component->ComponentTags) | std::views::transform([](const auto& tag) { return toStdString(tag); }));
}

//
// String functions
//

std::string Unreal::toStdString(const FName& str)
{
    return toStdString(str.ToString()); // str.ToString() converts FName to FString
}

std::string Unreal::toStdString(const FString& str)
{
    return std::string(TCHAR_TO_UTF8(*str)); // the * operator for FString returns a pointer to the underlying TCHAR array
}

std::string Unreal::toStdString(const FText& str)
{
    return toStdString(str.ToString()); // str.ToString() converts FText to FString
}

std::string Unreal::toStdString(const TCHAR* str)
{
    SP_ASSERT(str);
    return std::string(TCHAR_TO_UTF8(str));
}

FName Unreal::toFName(const std::string& str)
{
    return FName(str.c_str());
}

FString Unreal::toFString(const std::string& str)
{
    return FString(UTF8_TO_TCHAR(str.c_str()));
}

FText Unreal::toFText(const std::string& str)
{
    return FText::FromString(toFString(str));
}
