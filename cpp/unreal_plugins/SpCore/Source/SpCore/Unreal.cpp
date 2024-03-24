//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/Unreal.h"

#include <stddef.h> // size_t

#include <map>
#include <ranges> // std::views::transform
#include <string>
#include <utility> // std::move
#include <vector>

#include <Components/ActorComponent.h>
#include <Containers/Array.h>        // TArray
#include <Containers/StringConv.h>   // TCHAR_TO_UTF8, UTF8_TO_TCHAR
#include <Containers/UnrealString.h> // FString::operator*
#include <Dom/JsonObject.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // TCHAR
#include <JsonObjectConverter.h>
#include <Serialization/JsonReader.h>
#include <Serialization/JsonSerializer.h>
#include <Templates/SharedPointer.h> // TSharedPtr, TSharedRef
#include <UObject/Class.h>           // EIncludeSuperFlag, UClass, UStruct
#include <UObject/NameTypes.h>       // FName
#include <UObject/Object.h>          // UObject
#include <UObject/ObjectMacros.h>    // EPropertyFlags
#include <UObject/UnrealType.h>      // FBoolProperty, FDoubleProperty, FFloatProperty, FIntProperty, FProperty, FStrProperty, FStructProperty, TFieldIterator

#include "SpCore/Assert.h"
#include "SpCore/EngineActor.h"
#include "SpCore/Log.h"
#include "SpCore/StableNameComponent.h"
#include "SpCore/Std.h"

//
// String functions
//

std::string Unreal::toStdString(const FString& str)
{
    // Note that the * operator for FString returns a pointer to the underlying string
    return std::string(TCHAR_TO_UTF8(*str));
}

std::string Unreal::toStdString(const FName& str)
{
    // Note that str.ToString() converts FName to FString
    return toStdString(str.ToString());
}

std::string Unreal::toStdString(const TCHAR* str)
{
    return std::string(TCHAR_TO_UTF8(str));
}

FString Unreal::toFString(const std::string& str)
{
    return FString(UTF8_TO_TCHAR(str.c_str()));
}

FName Unreal::toFName(const std::string& str)
{
    return FName(str.c_str());
}

//
// Helper functions to get actor and component names.
//

std::string Unreal::getStableActorName(const AActor* actor)
{
    SP_ASSERT(actor);
    UStableNameComponent* stable_name_component = getComponentByType<UStableNameComponent>(actor);
    return toStdString(stable_name_component->StableName);
}

void Unreal::setStableActorName(const AActor* actor, std::string stable_name)
{
    SP_ASSERT(actor);
    UStableNameComponent* stable_name_component = getComponentByType<UStableNameComponent>(actor);
    stable_name_component->StableName = toFString(stable_name);
}

#if WITH_EDITOR // defined in an auto-generated header
    void Unreal::requestUpdateStableActorName(const AActor* actor)
    {
        SP_ASSERT(actor);
        bool assert_if_not_found = false;
        UStableNameComponent* stable_name_component = getComponentByType<UStableNameComponent>(actor, assert_if_not_found);
        if (stable_name_component) {
            stable_name_component->requestUpdate();
        }
    }
#endif

// 
// Find actors unconditionally and return an std::vector or an std::map
//

std::vector<AActor*> Unreal::findActors(const UWorld* world)
{
    return findActorsByType(world);
}

std::map<std::string, AActor*> Unreal::findActorsAsMap(const UWorld* world)
{
    return findActorsByTypeAsMap(world);
}

// 
// Get components unconditionally and return an std::vector or an std::map
//

std::vector<UActorComponent*> Unreal::getComponents(const AActor* actor)
{
    return getComponentsByType(actor);
}

std::map<std::string, UActorComponent*> Unreal::getComponentsAsMap(const AActor* actor)
{
    return getComponentsByTypeAsMap(actor);
}

//
// Helper functions for finding actors and getting components
//

bool Unreal::getActorHasStableName(const AActor* actor)
{
    SP_ASSERT(actor);
    bool assert_if_not_found = false;
    UStableNameComponent* stable_name_component = getComponentByType<UStableNameComponent>(actor, assert_if_not_found);
    return stable_name_component != nullptr;
}

std::vector<bool> Unreal::getActorHasTags(const AActor* actor, const std::vector<std::string>& tags)
{
    SP_ASSERT(actor);
    return Std::toVector<bool>(tags | std::views::transform([actor](const auto& tag) { return actor->ActorHasTag(toFName(tag)); }));
}

std::vector<bool> Unreal::getComponentHasTags(const UActorComponent* component, const std::vector<std::string>& tags)
{
    SP_ASSERT(component);
    return Std::toVector<bool>(tags | std::views::transform([component](const auto& tag) { return component->ComponentHasTag(toFName(tag)); }));
}

//
// Find struct by name and return UStruct*
//

UStruct* Unreal::findStructByName(const UWorld* world, const std::string& name)
{
    AEngineActor* engine_actor = findActorByType<AEngineActor>(world);
    SP_ASSERT(engine_actor);
    PropertyDesc property_desc = findPropertyByName(engine_actor, name);
    SP_ASSERT(property_desc.property_);
    SP_ASSERT(property_desc.property_->IsA(FStructProperty::StaticClass()));
    FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
    return struct_property->Struct;
}

//
// Find property by name and return a PropertyDesc
//

Unreal::PropertyDesc Unreal::findPropertyByName(UObject* uobject, const std::string& name)
{
    return findPropertyByName(uobject, uobject->GetClass(), name);
}

Unreal::PropertyDesc Unreal::findPropertyByName(void* value_ptr, const UStruct* ustruct, const std::string& name)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);

    std::vector<std::string> property_names = Std::tokenize(name, ".");

    PropertyDesc property_desc;
    property_desc.value_ptr_ = value_ptr;

    for (int i = 0; i < property_names.size() - 1; i++) {
        std::string& property_name = property_names.at(i);

        property_desc.property_ = ustruct->FindPropertyByName(Unreal::toFName(property_name));
        SP_ASSERT(property_desc.property_);

        property_desc.value_ptr_ = property_desc.property_->ContainerPtrToValuePtr<void>(property_desc.value_ptr_);
        SP_ASSERT(property_desc.value_ptr_);

        if (property_desc.property_->IsA(FStructProperty::StaticClass())) {
            FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
            ustruct = struct_property->Struct;
        } else {
            SP_LOG(property_name, " is an unsupported type:", toStdString(property_desc.property_->GetClass()->GetName()));
            SP_ASSERT(false);
        }
    }

    std::string& property_name = property_names.at(property_names.size() - 1);

    property_desc.property_ = ustruct->FindPropertyByName(Unreal::toFName(property_name));
    SP_ASSERT(property_desc.property_);

    property_desc.value_ptr_ = property_desc.property_->ContainerPtrToValuePtr<void>(property_desc.value_ptr_);
    SP_ASSERT(property_desc.value_ptr_);

    return property_desc;
}

//
// Get property value
//

std::string Unreal::getPropertyValueAsString(UObject* uobject)
{
    return getPropertyValueAsString(uobject, uobject->GetClass());
}

std::string Unreal::getPropertyValueAsString(void* value_ptr, const UStruct* ustruct)
{
    FString string;
    FJsonObjectConverter::UStructToJsonObjectString(ustruct, value_ptr, string);
    return toStdString(string);
}

std::string Unreal::getPropertyValueAsString(const Unreal::PropertyDesc& property_desc)
{
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.property_);

    if (property_desc.property_->IsA(FBoolProperty::StaticClass()) ||
        property_desc.property_->IsA(FIntProperty::StaticClass()) ||
        property_desc.property_->IsA(FFloatProperty::StaticClass()) ||
        property_desc.property_->IsA(FDoubleProperty::StaticClass()) ||
        property_desc.property_->IsA(FStrProperty::StaticClass())) {

        TSharedPtr<FJsonValue> json_value = FJsonObjectConverter::UPropertyToJsonValue(property_desc.property_, property_desc.value_ptr_);
        SP_ASSERT(json_value.Get());
        return toStdString(json_value.Get()->AsString());

    } else if (property_desc.property_->IsA(FStructProperty::StaticClass())) {
        FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
        UStruct* ustruct = struct_property->Struct;
        FString string;
        FJsonObjectConverter::UStructToJsonObjectString(ustruct, property_desc.value_ptr_, string);
        return toStdString(string);

    } else {
        SP_LOG(toStdString(property_desc.property_->GetName()), " is an unsupported type: ", toStdString(property_desc.property_->GetClass()->GetName()));
        SP_ASSERT(false);
        return "";
    }
}

//
// Initialize property to a default value
//

void Unreal::initializePropertyValue(UObject* uobject)
{
    return initializePropertyValue(uobject, uobject->GetClass());
}

void Unreal::initializePropertyValue(void* value_ptr, UStruct* ustruct)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);

    bool success = false;
    TSharedRef<TJsonReader<>> json_reader = TJsonReaderFactory<>::Create(toFString("{}"));
    TSharedPtr<FJsonObject> json_object;
    success = FJsonSerializer::Deserialize(json_reader, json_object);
    SP_ASSERT(success);
    SP_ASSERT(json_object.IsValid());
    success = FJsonObjectConverter::JsonObjectToUStruct(json_object.ToSharedRef(), ustruct, value_ptr);
    SP_ASSERT(success);
}

void Unreal::initializePropertyValue(const Unreal::PropertyDesc& property_desc)
{
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.property_);

    if (property_desc.property_->IsA(FBoolProperty::StaticClass())) {
        setPropertyValueFromString(property_desc, "false");

    } else if (
        property_desc.property_->IsA(FIntProperty::StaticClass()) ||
        property_desc.property_->IsA(FFloatProperty::StaticClass()) ||
        property_desc.property_->IsA(FDoubleProperty::StaticClass())) {
        setPropertyValueFromString(property_desc, "0");

    } else if (property_desc.property_->IsA(FStrProperty::StaticClass())) {
        setPropertyValueFromString(property_desc, "\"\"");
    
    } else if (property_desc.property_->IsA(FStructProperty::StaticClass())) {
        setPropertyValueFromString(property_desc, "{}");

    } else {
        SP_LOG(toStdString(property_desc.property_->GetName()), " is an unsupported type: ", toStdString(property_desc.property_->GetClass()->GetName()));
        SP_ASSERT(false);
    }
}

//
// Set property value
//

void Unreal::setPropertyValueFromString(UObject* uobject, const std::string& string)
{
    return setPropertyValueFromString(uobject, uobject->GetClass(), string);
}

void Unreal::setPropertyValueFromString(void* value_ptr, UStruct* ustruct, const std::string& string)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);

    bool success = false;
    TSharedRef<TJsonReader<>> json_reader = TJsonReaderFactory<>::Create(toFString(string));
    TSharedPtr<FJsonObject> json_object;
    success = FJsonSerializer::Deserialize(json_reader, json_object);
    SP_ASSERT(success);
    SP_ASSERT(json_object.IsValid());
    success = FJsonObjectConverter::JsonObjectToUStruct(json_object.ToSharedRef(), ustruct, value_ptr);
    SP_ASSERT(success);
}

void Unreal::setPropertyValueFromString(const Unreal::PropertyDesc& property_desc, const std::string& string)
{
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.property_);

    if (property_desc.property_->IsA(FBoolProperty::StaticClass()) ||
        property_desc.property_->IsA(FIntProperty::StaticClass()) ||
        property_desc.property_->IsA(FFloatProperty::StaticClass()) ||
        property_desc.property_->IsA(FDoubleProperty::StaticClass()) ||
        property_desc.property_->IsA(FStrProperty::StaticClass())) {

        bool success = false;
        TSharedRef<TJsonReader<>> json_reader = TJsonReaderFactory<>::Create(toFString("{ \"dummy\": " + string + "}"));
        TSharedPtr<FJsonObject> json_object;
        success = FJsonSerializer::Deserialize(json_reader, json_object);
        SP_ASSERT(success);
        SP_ASSERT(json_object.IsValid());
        TSharedPtr<FJsonValue> json_value = json_object.Get()->TryGetField("dummy");
        SP_ASSERT(json_value.Get());
        success = FJsonObjectConverter::JsonValueToUProperty(json_value, property_desc.property_, property_desc.value_ptr_);
        SP_ASSERT(success);

    } else if (property_desc.property_->IsA(FStructProperty::StaticClass())) {

        FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
        UStruct* ustruct = struct_property->Struct;
        setPropertyValueFromString(property_desc.value_ptr_, ustruct, string);

    } else {
        SP_LOG(toStdString(property_desc.property_->GetName()), " is an unsupported type: ", toStdString(property_desc.property_->GetClass()->GetName()));
        SP_ASSERT(false);
    }
}

//
// Find function by name and return a UFunction*
//

UFunction* Unreal::findFunctionByName(UClass* uclass, const std::string& name, EIncludeSuperFlag::Type include_super_flag)
{
    UFunction* function = uclass->FindFunctionByName(toFName(name), include_super_flag);
    SP_ASSERT(function);
    return function;
}

//
// Call function
//

std::map<std::string, std::string> Unreal::callFunction(UObject* uobject, UFunction* ufunction, const std::map<std::string, std::string>& args)
{
    // Create buffer to store all args and the return value.
    size_t num_bytes = ufunction->ParmsSize;
    uint8_t initial_value = 0;
    std::vector<uint8_t> args_vector(num_bytes, initial_value);

    // Create PropertyDescs for the function's arguments and return value.
    std::vector<PropertyDesc> property_descs;
    for (TFieldIterator<FProperty> itr(ufunction); itr; ++itr) {
        PropertyDesc property_desc;
        property_desc.property_ = *itr;
        SP_ASSERT(property_desc.property_);
        SP_ASSERT(property_desc.property_->HasAnyPropertyFlags(EPropertyFlags::CPF_Parm));

        property_desc.value_ptr_ = property_desc.property_->ContainerPtrToValuePtr<void>(args_vector.data());
        SP_ASSERT(property_desc.value_ptr_);

        property_descs.push_back(std::move(property_desc));
    }

    // Set property values.
    for (auto& property_desc : property_descs) {
        std::string property_name = toStdString(property_desc.property_->GetName());
        initializePropertyValue(property_desc);
        if (Std::containsKey(args, property_name)) {
            setPropertyValueFromString(property_desc, args.at(property_name));
        }
    }

    // Call function.
    uobject->ProcessEvent(ufunction, args_vector.data());

    // Return all property values because they might have been modified by the function we called.
    std::map<std::string, std::string> return_values;
    for (auto& property_desc : property_descs) {
        std::string property_name = toStdString(property_desc.property_->GetName());
        Std::insert(return_values, property_name, getPropertyValueAsString(property_desc));
    }

    return return_values;
}
