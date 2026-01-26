//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/UnrealUtils.h"

#include <cstdlib> // std::atoi
#include <map>
#include <ranges>  // std::views::filter, std::views::transform
#include <string>
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>
#include <Containers/UnrealString.h>    // FString::operator*
#include <Dom/JsonObject.h>
#include <Dom/JsonValue.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>               // TCHAR, uint16
#include <JsonObjectConverter.h>
#include <Serialization/JsonReader.h>
#include <Serialization/JsonSerializer.h>
#include <Templates/Casts.h>
#include <Templates/SharedPointer.h>    // TSharedPtr, TSharedRef
#include <UObject/Class.h>              // EIncludeSuperFlag, UClass, UScriptStruct, UStruct
#include <UObject/NameTypes.h>          // FName
#include <UObject/Object.h>             // UObject
#include <UObject/Script.h>             // EFunctionFlags
#include <UObject/ScriptInterface.h>    // FScriptInterface
#include <UObject/UnrealType.h>         // EFieldIterationFlags, FArrayProperty, FBoolProperty, FByteProperty, FDoubleProperty, FInt8Property, FInt16Property,
                                        // FInt64Property, FFloatProperty, FIntProperty, FMapProperty, FProperty, FScriptArrayHelper, FScriptMapHelper,
                                        // FScriptSetHelper, FSetProperty, FStrProperty, FStructProperty, FUInt16Property, FUInt32Property, FUInt64Property,
                                        // TFieldIterator
#include <UObject/UObjectHash.h>        // GetDerivedClasses

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpStableNameManager.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

class UWorld;

//
// Functions for finding static structs
//

std::vector<UScriptStruct*> UnrealUtils::findStaticStructs()
{
    return findStaticStructsByType<UScriptStruct>();
}

std::map<std::string, UScriptStruct*> UnrealUtils::findStaticStructsAsMap()
{
    return toMap(findStaticStructs());
}

//
// Functions for getting static classes
//

std::vector<UClass*> UnrealUtils::findStaticClasses()
{
    return findStaticStructsByType<UClass>();
}

std::map<std::string, UClass*> UnrealUtils::findStaticClassesAsMap()
{
    return toMap(findStaticClasses());
}

std::vector<UClass*> UnrealUtils::getDerivedClasses(UClass* uclass, bool recursive)
{
    TArray<UClass*> derived_uclasses;
    GetDerivedClasses(uclass, derived_uclasses, recursive);
    return Unreal::toStdVector(derived_uclasses);
}

std::map<std::string, UClass*> UnrealUtils::getDerivedClassesAsMap(UClass* uclass, bool recursive)
{
    return toMap(getDerivedClasses(uclass, recursive));
}

//
// Find functions
//

std::vector<UFunction*> UnrealUtils::findFunctions(const UClass* uclass, EFieldIterationFlags field_iteration_flags)
{
    SP_ASSERT(uclass);
    std::vector<UFunction*> ufunctions;
    for (TFieldIterator<UFunction> itr(uclass, field_iteration_flags); itr; ++itr) {
        UFunction* ufunction = *itr;
        ufunctions.push_back(ufunction);
    }
    return ufunctions;
}

std::map<std::string, UFunction*> UnrealUtils::findFunctionsAsMap(const UClass* uclass, EFieldIterationFlags field_iteration_flags)
{
    return toMap(findFunctions(uclass, field_iteration_flags));
}

std::vector<UFunction*> UnrealUtils::findFunctionsByName(const UClass* uclass, const std::string& function_name, EFieldIterationFlags field_iteration_flags)
{
    return Std::toVector<UFunction*>(
        findFunctions(uclass, field_iteration_flags) |
        std::views::filter([&function_name](auto function) { return Unreal::toStdString(function->GetName()) == function_name; }));
}

std::vector<UFunction*> UnrealUtils::findFunctionsByFlagsAny(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags)
{
    return Std::toVector<UFunction*>(
        findFunctions(uclass, field_iteration_flags) |
        std::views::filter([function_flags](auto function) { return function->HasAnyFunctionFlags(function_flags); }));
}

std::vector<UFunction*> UnrealUtils::findFunctionsByFlagsAll(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags)
{
    return Std::toVector<UFunction*>(
        findFunctions(uclass, field_iteration_flags) |
        std::views::filter([function_flags](auto function) { return function->HasAllFunctionFlags(function_flags); }));
}

std::map<std::string, UFunction*> UnrealUtils::findFunctionsByNameAsMap(const UClass* uclass, const std::string& function_name, EFieldIterationFlags field_iteration_flags)
{
    return toMap(findFunctionsByName(uclass, function_name, field_iteration_flags));
}

std::map<std::string, UFunction*> UnrealUtils::findFunctionsByFlagsAnyAsMap(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags)
{
    return toMap(findFunctionsByFlagsAny(uclass, function_flags, field_iteration_flags));
}

std::map<std::string, UFunction*> UnrealUtils::findFunctionsByFlagsAllAsMap(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags)
{
    return toMap(findFunctionsByFlagsAll(uclass, function_flags, field_iteration_flags));
}

UFunction* UnrealUtils::findFunctionByName(const UClass* uclass, const std::string& function_name, EFieldIterationFlags field_iteration_flags)
{
    return toItem(findFunctionsByName(uclass, function_name, field_iteration_flags));
}

UFunction* UnrealUtils::findFunctionByFlagsAny(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags)
{
    return toItem(findFunctionsByFlagsAny(uclass, function_flags, field_iteration_flags));
}

UFunction* UnrealUtils::findFunctionByFlagsAll(const UClass* uclass, EFunctionFlags function_flags, EFieldIterationFlags field_iteration_flags)
{
    return toItem(findFunctionsByFlagsAll(uclass, function_flags, field_iteration_flags));
}

//
// Call function
//

std::map<std::string, SpPropertyValue> UnrealUtils::callFunction(const UWorld* world, UObject* uobject, UFunction* ufunction, const std::map<std::string, std::string>& args, const std::string& world_context_object)
{
    SP_ASSERT(world);
    SP_ASSERT(uobject);
    SP_ASSERT(ufunction);

    // Create buffer to store all args and the return value.
    uint16 num_bytes = ufunction->ParmsSize;
    uint8_t initial_value = 0;
    std::vector<uint8_t> args_vector(num_bytes, initial_value);

    // Create SpPropertyDescs for the function's arguments and return value.
    std::map<std::string, SpPropertyDesc> property_descs;
    for (TFieldIterator<FProperty> itr(ufunction); itr; ++itr) {
        FProperty* property = *itr;

        if (property->HasAnyPropertyFlags(EPropertyFlags::CPF_Parm)) {
            SpPropertyDesc property_desc;
            property_desc.property_ = property;
            SP_ASSERT(property_desc.property_);
            property_desc.type_id_ = Unreal::getCppTypeAsString(property_desc.property_);
            SP_ASSERT(property_desc.type_id_ != "");
            property_desc.value_ptr_ = property_desc.property_->ContainerPtrToValuePtr<void>(args_vector.data());
            SP_ASSERT(property_desc.value_ptr_);

            std::string property_name = Unreal::toStdString(property_desc.property_->GetName());
            Std::insert(property_descs, std::move(property_name), std::move(property_desc));
        }
    }

    // Explicitly initialize if an arg or return value requires it.
    for (auto& [property_name, property_desc] : property_descs) {
        if (!property_desc.property_->HasAnyPropertyFlags(EPropertyFlags::CPF_ZeroConstructor)) {
            property_desc.property_->InitializeValue_InContainer(args_vector.data());
        }
    }

    // Input args must be a subset of the function's args.
    SP_ASSERT(Std::isSubsetOf(Std::keys(args), Std::keys(property_descs)));

    // Set property values.
    for (auto& [property_name, property_desc] : property_descs) {

        // If the current property name has been flagged by the caller as being the special world_context_object
        // arg, then set the property using the input world pointer instead of using a string in args, and
        // make sure the property name is not also present in args. Strictly speaking, we could assign the
        // world pointer directly to *(property_desc.value_ptr), but this could lead to undefined behavior
        // if the property is not actually a UObject pointer. So we assign via setPropertValueFromString(...),
        // because it provides well-defined behavior in all cases. Either the assignment is possible
        // according to Unreal's JSON assignment rules, in which case setPropertValueFromString(...) will
        // perform the assignment, or the assignment is not possible, in which case setPropertValueFromString(...)
        // will assert.

        if (property_name == world_context_object) {
            SP_ASSERT(!Std::containsKey(args, property_name));
            setPropertyValueFromString(property_desc, Std::toStringFromPtr(world));
        }

        // If the property name is in args, then set it using the string in args, and make sure the property
        // name was not also flagged by the caller as being the special world_context_object arg.

        if (Std::containsKey(args, property_name)) {
            SP_ASSERT(property_name != world_context_object);
            setPropertyValueFromString(property_desc, args.at(property_name));
        }
    }

    // Call function.
    uobject->ProcessEvent(ufunction, args_vector.data());

    // Return all property values because they might have been modified by the function we called.
    std::map<std::string, SpPropertyValue> return_values;
    for (auto& [property_name, property_desc] : property_descs) {
        Std::insert(return_values, property_name, getPropertyValueAsString(property_desc));
    }

    // Explicitly destroy if an arg or return value requires it.
    for (auto& [property_name, property_desc] : property_descs) {
        if (!property_desc.property_->HasAnyPropertyFlags(EPropertyFlags::CPF_ZeroConstructor)) {
            property_desc.property_->DestroyValue_InContainer(args_vector.data());
        }
    }

    return return_values;
}

//
// Find properties (could be templated but we choose to explicitly instantiate for readability)
//

std::vector<FProperty*> UnrealUtils::findProperties(const UStruct* ustruct, EFieldIterationFlags field_iteration_flags)
{
    return findPropertiesAll(ustruct, field_iteration_flags);
}

std::vector<FProperty*> UnrealUtils::findProperties(const UFunction* ufunction, EFieldIterationFlags field_iteration_flags)
{
    return findPropertiesAll(ufunction, field_iteration_flags);
}

//
// Find actors unconditionally and return an std::vector or an std::map
//

std::vector<AActor*> UnrealUtils::findActors(const UWorld* world)
{
    return findActorsByType(world);
}

std::map<std::string, AActor*> UnrealUtils::findActorsAsMap(const UWorld* world)
{
    return toMap(findActors(world));
}

// 
// Get components unconditionally and return an std::vector or an std::map
//

std::vector<UActorComponent*> UnrealUtils::getComponents(const AActor* actor)
{
    return getComponentsByType(actor);
}

std::map<std::string, UActorComponent*> UnrealUtils::getComponentsAsMap(const AActor* actor)
{
    return toMap(getComponents(actor));
}

//
// Get children components unconditionally and return an std::vector or an std::map (could be templated but we choose to explicitly instantiate for readability)
//

std::vector<USceneComponent*> UnrealUtils::getChildrenComponents(const AActor* parent, bool include_all_descendants)
{
    return getChildrenComponentsByType(parent, include_all_descendants);
}

std::vector<USceneComponent*> UnrealUtils::getChildrenComponents(const USceneComponent* parent, bool include_all_descendants)
{
    return getChildrenComponentsByType(parent, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealUtils::getChildrenComponentsAsMap(const AActor* parent, bool include_all_descendants)
{
    return toMap(getChildrenComponents(parent, include_all_descendants));
}

std::map<std::string, USceneComponent*> UnrealUtils::getChildrenComponentsAsMap(const USceneComponent* parent, bool include_all_descendants)
{
    return toMap(getChildrenComponents(parent, include_all_descendants));
}

//
// Get and set object properties, uobject can't be const because we cast it to void*
//

std::string UnrealUtils::getObjectPropertiesAsString(const UObject* uobject)
{
    return getObjectPropertiesAsString(uobject, uobject->GetClass());
}

std::string UnrealUtils::getObjectPropertiesAsString(const void* value_ptr, const UStruct* ustruct)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);
    FString string;
    FJsonObjectConverter::UStructToJsonObjectString(ustruct, value_ptr, string);
    return Unreal::toStdString(string);
}

void UnrealUtils::setObjectPropertiesFromString(UObject* uobject, const std::string& string)
{
    SP_ASSERT(uobject);
    return setObjectPropertiesFromString(uobject, uobject->GetClass(), string);
}

void UnrealUtils::setObjectPropertiesFromString(void* value_ptr, const UStruct* ustruct, const std::string& string)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);
    bool success = false;
    TSharedRef<TJsonReader<>> json_reader = TJsonReaderFactory<>::Create(Unreal::toFString(string));
    TSharedPtr<FJsonObject> json_object;
    success = FJsonSerializer::Deserialize(json_reader, json_object);
    SP_ASSERT(success);
    SP_ASSERT(json_object.IsValid());
    success = FJsonObjectConverter::JsonObjectToUStruct(json_object.ToSharedRef(), ustruct, value_ptr);
    SP_ASSERT(success);
}

//
// Find property by name, get and set property values, uobject can't be const because we cast it to void*,
// value_ptr can't be const because we assign to SpPropertyDesc::value_ptr_.
//

SpPropertyDesc UnrealUtils::findPropertyByName(UObject* uobject, const std::string& property_name)
{
    SP_ASSERT(uobject);
    return findPropertyByName(uobject, uobject->GetClass(), property_name);
}

SpPropertyDesc UnrealUtils::findPropertyByName(void* value_ptr, const UStruct* ustruct, const std::string& property_name)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);

    std::vector<std::string> property_names = Std::tokenize(property_name, ".");

    SpPropertyDesc property_desc;
    property_desc.value_ptr_ = value_ptr;

    for (int i = 0; i < property_names.size(); i++) {

        std::string& name = property_names.at(i);
        std::vector<std::string> name_tokens = Std::tokenize(name, "[]");
        SP_ASSERT(name_tokens.size() >= 1 && name_tokens.size() <= 2);

        property_desc.property_ = ustruct->FindPropertyByName(Unreal::toFName(name_tokens.at(0)));
        SP_ASSERT(property_desc.property_);
        property_desc.type_id_ = Unreal::getCppTypeAsString(property_desc.property_);
        SP_ASSERT(property_desc.type_id_ != "");
        property_desc.value_ptr_ = property_desc.property_->ContainerPtrToValuePtr<void>(property_desc.value_ptr_);
        SP_ASSERT(property_desc.value_ptr_);

        // If the current property is an array or map property, and the name includes the index operator,
        // then update the current property to refer to the array or map element based on the index. For map
        // properties, we expect an index string that is enclosed enclosed in "" quotes if the key type is a
        // string, and not enclosed in quotes otherwise. The index string must exactly match whatever is
        // returned by getPropertyValueAsString(...) for the key.

        if (property_desc.property_->IsA(FArrayProperty::StaticClass()) && name_tokens.size() == 2) {

            FArrayProperty* array_property = static_cast<FArrayProperty*>(property_desc.property_);
            FScriptArrayHelper array_helper(array_property, property_desc.value_ptr_);

            property_desc.property_ = array_property->Inner;
            SP_ASSERT(property_desc.property_);
            property_desc.type_id_ = Unreal::getCppTypeAsString(property_desc.property_);
            SP_ASSERT(property_desc.type_id_ != "");

            int index = std::atoi(name_tokens.at(1).c_str());
            SP_ASSERT(index < array_helper.Num());
            property_desc.value_ptr_ = array_property->GetValueAddressAtIndex_Direct(property_desc.property_, property_desc.value_ptr_, index);
            SP_ASSERT(property_desc.value_ptr_);

        } else if (property_desc.property_->IsA(FMapProperty::StaticClass()) && name_tokens.size() == 2) {

            FMapProperty* map_property = static_cast<FMapProperty*>(property_desc.property_);
            FScriptMapHelper map_helper(map_property, property_desc.value_ptr_);

            property_desc.property_ = map_property->ValueProp;
            SP_ASSERT(property_desc.property_);
            property_desc.type_id_ = Unreal::getCppTypeAsString(property_desc.property_);
            SP_ASSERT(property_desc.type_id_ != "");

            bool found = false;
            for (int j = 0; j < map_helper.Num(); j++) {
                SpPropertyDesc inner_key_property_desc;
                inner_key_property_desc.property_ = map_property->KeyProp;
                SP_ASSERT(inner_key_property_desc.property_);
                inner_key_property_desc.type_id_ = Unreal::getCppTypeAsString(inner_key_property_desc.property_);
                SP_ASSERT(inner_key_property_desc.type_id_ != "");
                inner_key_property_desc.value_ptr_ = map_property->GetValueAddressAtIndex_Direct(map_property->KeyProp, property_desc.value_ptr_, j);
                SP_ASSERT(inner_key_property_desc.value_ptr_);

                SpPropertyValue inner_key_property_value = getPropertyValueAsString(inner_key_property_desc);
                std::string inner_key_string = inner_key_property_value.value_;

                // If the key type is a string, then we expect the index string to be enclosed in quotes,
                // otherwise we expect it not to be enclosed in quotes.
                if (inner_key_property_desc.property_->IsA(FBoolProperty::StaticClass()) ||
                    inner_key_property_desc.property_->IsA(FIntProperty::StaticClass()) ||
                    inner_key_property_desc.property_->IsA(FByteProperty::StaticClass()) ||
                    inner_key_property_desc.property_->IsA(FEnumProperty::StaticClass())) {
                    if (name_tokens.at(1) == inner_key_string) {
                        found = true;
                    }
                } else if (inner_key_property_desc.property_->IsA(FStrProperty::StaticClass())) {
                    if (name_tokens.at(1) == "\"" + inner_key_string + "\"") {
                        found = true;
                    }
                } else {
                    SP_LOG(name, " has an unsupported key type: ", Unreal::toStdString(map_property->KeyProp->GetClass()->GetName()));
                    SP_ASSERT(false);
                }
                if (found) {
                    property_desc.value_ptr_ = map_property->GetValueAddressAtIndex_Direct(map_property->ValueProp, property_desc.value_ptr_, j);
                    SP_ASSERT(property_desc.value_ptr_);
                    break;
                }
            }
            SP_ASSERT(found);
        }

        // If the current property name is not the last name in our sequence, then by definition the current
        // property refers to something with named properties (i.e., a struct or an object). In this case,
        // we need to update our ustruct pointer, and potentially our value_ptr if our current property
        // refers to an object. In contrast, if the current property name is the last name in our sequence,
        // then there is nothing more to do, because we have already filled in our SpPropertyDesc object.

        if (i < property_names.size() - 1) {
            if (property_desc.property_->IsA(FObjectProperty::StaticClass())) {
                FObjectProperty* object_property = static_cast<FObjectProperty*>(property_desc.property_);
                UObject* uobject = object_property->GetObjectPropertyValue(property_desc.value_ptr_);
                SP_ASSERT(uobject);
                property_desc.value_ptr_ = uobject;
                ustruct = uobject->GetClass();

            } else if (property_desc.property_->IsA(FStructProperty::StaticClass())) {
                FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
                ustruct = struct_property->Struct;

            } else {
                SP_LOG(name, " is an unsupported type: ", Unreal::toStdString(property_desc.property_->GetClass()->GetName()));
                SP_ASSERT(false);
            }
        }
    }

    return property_desc;
}

SpPropertyValue UnrealUtils::getPropertyValueAsString(const SpPropertyDesc& property_desc)
{
    SP_ASSERT(property_desc.property_);
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.type_id_ != "");

    SpPropertyValue property_value;

    // Set type_id_ from the inpput desc
    property_value.type_id_ = property_desc.type_id_;

    // We need a special case for FStructProperty in this code block, but not in setPropertyValueFromString(...)
    // because we rely on FJsonValue::TryGetString(...) here, and this function does not work for structs. On
    // the other hand, the equivalent code block in setPropertyValueFromString(...) doesn't need to call
    // TryGetString(), so it also behaves correctly for structs.

    // See Engine/Source/Editor/Kismet/Internal/Reflection/FunctionUtilsPrivate.h for an exaustive
    // enumeration of all possible property types. For now, we only handle the property types that have come
    // up in practice in our experiments.

    if (property_desc.property_->IsA(FBoolProperty::StaticClass()) ||
        property_desc.property_->IsA(FIntProperty::StaticClass()) ||
        property_desc.property_->IsA(FInt8Property::StaticClass()) ||
        property_desc.property_->IsA(FInt16Property::StaticClass()) ||
        // no FInt32Property because the int32 C++ type maps to FIntProperty
        property_desc.property_->IsA(FInt64Property::StaticClass()) ||
        // no FUInt8Property because the uint8 C++ type maps to FByteProperty
        property_desc.property_->IsA(FUInt16Property::StaticClass()) ||
        property_desc.property_->IsA(FUInt32Property::StaticClass()) ||
        property_desc.property_->IsA(FUInt64Property::StaticClass()) ||
        property_desc.property_->IsA(FFloatProperty::StaticClass()) ||
        property_desc.property_->IsA(FDoubleProperty::StaticClass()) ||
        property_desc.property_->IsA(FStrProperty::StaticClass()) ||
        property_desc.property_->IsA(FNameProperty::StaticClass()) ||
        property_desc.property_->IsA(FByteProperty::StaticClass()) ||
        property_desc.property_->IsA(FEnumProperty::StaticClass()) ||
        property_desc.property_->IsA(FSoftObjectProperty::StaticClass())) {

        TSharedPtr<FJsonValue> json_value = FJsonObjectConverter::UPropertyToJsonValue(property_desc.property_, property_desc.value_ptr_);
        SP_ASSERT(json_value.IsValid());

        bool success = false;
        FString fstring;
        success = json_value.Get()->TryGetString(fstring);
        SP_ASSERT(success);

        property_value.value_ = Unreal::toStdString(fstring);

    }  else if (property_desc.property_->IsA(FArrayProperty::StaticClass())) {

        FArrayProperty* array_property = static_cast<FArrayProperty*>(property_desc.property_);
        FScriptArrayHelper array_helper(array_property, property_desc.value_ptr_);

        FProperty* inner_property = array_property->Inner;
        SP_ASSERT(inner_property);

        std::vector<std::string> inner_strings;
        for (int i = 0; i < array_helper.Num(); i++) {
            SpPropertyDesc inner_property_desc;
            inner_property_desc.property_ = inner_property;
            inner_property_desc.type_id_ = Unreal::getCppTypeAsString(inner_property_desc.property_);
            SP_ASSERT(inner_property_desc.type_id_ != "");
            inner_property_desc.value_ptr_ = array_property->GetValueAddressAtIndex_Direct(inner_property, property_desc.value_ptr_, i);
            SP_ASSERT(inner_property_desc.value_ptr_);
            std::string inner_string = getPropertyValueAsString(inner_property_desc).value_;
            inner_strings.push_back(inner_string);
        }

        property_value.value_ = getArrayPropertyValueAsFormattedString(inner_property, inner_strings);

    }  else if (property_desc.property_->IsA(FSetProperty::StaticClass())) {

        FSetProperty* set_property = static_cast<FSetProperty*>(property_desc.property_);
        FScriptSetHelper set_helper(set_property, property_desc.value_ptr_);

        FProperty* inner_property = set_property->ElementProp;
        SP_ASSERT(inner_property);

        std::vector<std::string> inner_strings;
        for (int i = 0; i < set_helper.Num(); i++) {
            SpPropertyDesc inner_property_desc;
            inner_property_desc.property_ = inner_property;
            inner_property_desc.type_id_ = Unreal::getCppTypeAsString(inner_property_desc.property_);
            SP_ASSERT(inner_property_desc.type_id_ != "");
            inner_property_desc.value_ptr_ = set_property->GetValueAddressAtIndex_Direct(inner_property, property_desc.value_ptr_, i);
            SP_ASSERT(inner_property_desc.value_ptr_);
            std::string inner_string = getPropertyValueAsString(inner_property_desc).value_;
            inner_strings.push_back(inner_string);
        }

        property_value.value_ = getArrayPropertyValueAsFormattedString(inner_property, inner_strings);

    }  else if (property_desc.property_->IsA(FMapProperty::StaticClass())) {

        FMapProperty* map_property = static_cast<FMapProperty*>(property_desc.property_);
        FScriptMapHelper map_helper(map_property, property_desc.value_ptr_);

        FProperty* inner_key_property = map_property->KeyProp;
        SP_ASSERT(inner_key_property);
        FProperty* inner_value_property = map_property->ValueProp;
        SP_ASSERT(inner_value_property);

        std::vector<std::string> inner_key_strings;
        std::vector<std::string> inner_value_strings;
        for (int i = 0; i < map_helper.Num(); i++) {
            SpPropertyDesc inner_key_property_desc;
            inner_key_property_desc.property_ = inner_key_property;
            inner_key_property_desc.type_id_ = Unreal::getCppTypeAsString(inner_key_property_desc.property_);
            SP_ASSERT(inner_key_property_desc.type_id_ != "");
            inner_key_property_desc.value_ptr_ = map_property->GetValueAddressAtIndex_Direct(inner_key_property, property_desc.value_ptr_, i);
            SP_ASSERT(inner_key_property_desc.value_ptr_);
            std::string inner_key_string = getPropertyValueAsString(inner_key_property_desc).value_;
            inner_key_strings.push_back(inner_key_string);

            SpPropertyDesc inner_value_property_desc;
            inner_value_property_desc.property_ = inner_value_property;
            inner_value_property_desc.type_id_ = Unreal::getCppTypeAsString(inner_value_property_desc.property_);
            SP_ASSERT(inner_value_property_desc.type_id_ != "");
            inner_value_property_desc.value_ptr_ = map_property->GetValueAddressAtIndex_Direct(inner_value_property, property_desc.value_ptr_, i);
            SP_ASSERT(inner_value_property_desc.value_ptr_);
            std::string inner_value_string = getPropertyValueAsString(inner_value_property_desc).value_;
            inner_value_strings.push_back(inner_value_string);
        }

        property_value.value_ = getMapPropertyValueAsFormattedString(inner_key_property, inner_key_strings, inner_value_property, inner_value_strings);

    } else if (property_desc.property_->IsA(FStructProperty::StaticClass())) {

        FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
        UStruct* ustruct = struct_property->Struct;
        FString fstring;
        FJsonObjectConverter::UStructToJsonObjectString(ustruct, property_desc.value_ptr_, fstring);
        property_value.value_ = Unreal::toStdString(fstring);

    } else if (property_desc.property_->IsA(FObjectProperty::StaticClass())) {

        FObjectProperty* object_property = static_cast<FObjectProperty*>(property_desc.property_);
        UObject* uobject = object_property->GetObjectPropertyValue(property_desc.value_ptr_);
        property_value.value_ = Std::toStringFromPtr(uobject);

    } else if (property_desc.property_->IsA(FInterfaceProperty::StaticClass())) {

        FInterfaceProperty* interface_property = static_cast<FInterfaceProperty*>(property_desc.property_);
        FScriptInterface* script_interface = interface_property->GetPropertyValuePtr(property_desc.value_ptr_);
        SP_ASSERT(script_interface);
        UObject* uobject = script_interface->GetObject();
        property_value.value_ = Std::toStringFromPtr(uobject);

    } else {

        SP_LOG(Unreal::toStdString(property_desc.property_->GetName()), " is an unsupported type: ", Unreal::toStdString(property_desc.property_->GetClass()->GetName()));
        SP_ASSERT(false);
    }

    return property_value;
}

void UnrealUtils::setPropertyValueFromString(const SpPropertyDesc& property_desc, const std::string& string)
{
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.property_);

    std::string quote_string = getQuoteStringForProperty(property_desc.property_);

    bool success = false;

    TSharedRef<TJsonReader<>> json_reader = TJsonReaderFactory<>::Create(Unreal::toFString("{ \"dummy\": " + quote_string + string + quote_string + " }"));
    TSharedPtr<FJsonObject> json_object;
    success = FJsonSerializer::Deserialize(json_reader, json_object);
    SP_ASSERT(success);
    SP_ASSERT(json_object.IsValid());
    TSharedPtr<FJsonValue> json_value = json_object.Get()->TryGetField(Unreal::toFString("dummy"));
    SP_ASSERT(json_value.IsValid());

    setPropertyValueFromJsonValue(property_desc, json_value);
}

void UnrealUtils::setPropertyValueFromJsonValue(const SpPropertyDesc& property_desc, TSharedPtr<FJsonValue> json_value)
{
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.property_);
    SP_ASSERT(json_value.IsValid());

    // We don't need a special case for FStructProperty in this code block, but we do in getPropertyValueAsString(...).
    // See the discussion above for details.

    // See Engine/Source/Editor/Kismet/Internal/Reflection/FunctionUtilsPrivate.h for an exaustive
    // enumeration of all possible property types. For now, we only handle the property types that have come
    // up in practice in our experiments.

    if (property_desc.property_->IsA(FBoolProperty::StaticClass()) ||
        property_desc.property_->IsA(FIntProperty::StaticClass()) ||
        property_desc.property_->IsA(FInt8Property::StaticClass()) ||
        property_desc.property_->IsA(FInt16Property::StaticClass()) ||
        // no FInt32Property because the int32 C++ type maps to FIntProperty
        property_desc.property_->IsA(FInt64Property::StaticClass()) ||
        // no FUInt8Property because the uint8 C++ type maps to FByteProperty
        property_desc.property_->IsA(FUInt16Property::StaticClass()) ||
        property_desc.property_->IsA(FUInt32Property::StaticClass()) ||
        property_desc.property_->IsA(FUInt64Property::StaticClass()) ||
        property_desc.property_->IsA(FFloatProperty::StaticClass()) ||
        property_desc.property_->IsA(FDoubleProperty::StaticClass()) ||
        property_desc.property_->IsA(FStrProperty::StaticClass()) ||
        property_desc.property_->IsA(FNameProperty::StaticClass()) ||
        property_desc.property_->IsA(FByteProperty::StaticClass()) ||
        property_desc.property_->IsA(FEnumProperty::StaticClass()) ||
        property_desc.property_->IsA(FStructProperty::StaticClass()) ||
        property_desc.property_->IsA(FSoftObjectProperty::StaticClass())) {

        bool success = false;
        success = FJsonObjectConverter::JsonValueToUProperty(json_value, property_desc.property_, property_desc.value_ptr_);
        SP_ASSERT(success);

    } else if (property_desc.property_->IsA(FArrayProperty::StaticClass())) {

        bool success = false;
        TArray<TSharedPtr<FJsonValue>>* json_values;
        success = json_value->TryGetArray(json_values);
        SP_ASSERT(success);
        SP_ASSERT(json_values);

        FArrayProperty* array_property = static_cast<FArrayProperty*>(property_desc.property_);
        FScriptArrayHelper array_helper(array_property, property_desc.value_ptr_);
        array_helper.Resize(json_values->Num());

        FProperty* inner_property = array_property->Inner;
        SP_ASSERT(inner_property);

        for (int i = 0; i < json_values->Num(); i++) {
            TSharedPtr<FJsonValue> inner_json_value = (*json_values)[i];
            SP_ASSERT(inner_json_value.IsValid());

            SpPropertyDesc inner_property_desc;
            inner_property_desc.property_ = inner_property;
            inner_property_desc.value_ptr_ = array_property->GetValueAddressAtIndex_Direct(inner_property, property_desc.value_ptr_, i);
            SP_ASSERT(inner_property_desc.value_ptr_);

            setPropertyValueFromJsonValue(inner_property_desc, inner_json_value);
        }

    } else if (property_desc.property_->IsA(FSetProperty::StaticClass())) {

        bool success = false;
        TArray<TSharedPtr<FJsonValue>>* json_values;
        success = json_value->TryGetArray(json_values);
        SP_ASSERT(success);

        FSetProperty* set_property = static_cast<FSetProperty*>(property_desc.property_);
        FScriptSetHelper set_helper(set_property, property_desc.value_ptr_);
        set_helper.EmptyElements(json_values->Num());

        FProperty* inner_property = set_property->ElementProp;
        SP_ASSERT(inner_property);

        for (int i = 0; i < json_values->Num(); i++) {
            TSharedPtr<FJsonValue> inner_json_value = (*json_values)[i];
            SP_ASSERT(inner_json_value.IsValid());

            int32 index = set_helper.AddDefaultValue_Invalid_NeedsRehash();

            SpPropertyDesc inner_property_desc;
            inner_property_desc.property_ = inner_property;
            inner_property_desc.value_ptr_ = set_helper.GetElementPtr(index);
            SP_ASSERT(inner_property_desc.value_ptr_);

            setPropertyValueFromJsonValue(inner_property_desc, inner_json_value);
        }

        set_helper.Rehash();

    }  else if (property_desc.property_->IsA(FMapProperty::StaticClass())) {

        bool success = false;
        TSharedPtr<FJsonObject>* json_object;
        success = json_value->TryGetObject(json_object);
        SP_ASSERT(success);
        SP_ASSERT(json_object->IsValid());
        TArray<FString> json_keys;
        TArray<TSharedPtr<FJsonValue>> json_values;
        json_object->Get()->Values.GenerateKeyArray(json_keys);
        json_object->Get()->Values.GenerateValueArray(json_values);
        SP_ASSERT(json_keys.Num() == json_values.Num());

        FMapProperty* map_property = static_cast<FMapProperty*>(property_desc.property_);
        FScriptMapHelper map_helper(map_property, property_desc.value_ptr_);
        map_helper.EmptyValues(json_object->Get()->Values.Num());

        FProperty* inner_key_property = map_property->KeyProp;
        SP_ASSERT(inner_key_property);
        FProperty* inner_value_property = map_property->ValueProp;
        SP_ASSERT(inner_value_property);

        for (int i = 0; i < json_values.Num(); i++) {
            std::string json_key = Unreal::toStdString(json_keys[i]);

            int32 index = map_helper.AddDefaultValue_Invalid_NeedsRehash();

            SpPropertyDesc inner_key_property_desc;
            inner_key_property_desc.property_ = inner_key_property;
            inner_key_property_desc.value_ptr_ = map_helper.GetKeyPtr(index);
            SP_ASSERT(inner_key_property_desc.value_ptr_);

            setPropertyValueFromString(inner_key_property_desc, json_key);

            TSharedPtr<FJsonValue> inner_json_value = json_values[i];
            SP_ASSERT(inner_json_value.IsValid());

            SpPropertyDesc inner_value_property_desc;
            inner_value_property_desc.property_ = inner_value_property;
            inner_value_property_desc.value_ptr_ = map_helper.GetValuePtr(index);
            SP_ASSERT(inner_value_property_desc.value_ptr_);

            setPropertyValueFromJsonValue(inner_value_property_desc, inner_json_value);
        }

        map_helper.Rehash();

    } else if (property_desc.property_->IsA(FObjectProperty::StaticClass())) {

        FObjectProperty* object_property = static_cast<FObjectProperty*>(property_desc.property_);

        bool success = false;
        FString fstring;
        success = json_value.Get()->TryGetString(fstring);
        SP_ASSERT(success);

        UObject* uobject = Std::toPtrFromString<UObject>(Unreal::toStdString(fstring));
        object_property->SetObjectPropertyValue(property_desc.value_ptr_, uobject);

    } else if (property_desc.property_->IsA(FInterfaceProperty::StaticClass())) {

        FInterfaceProperty* interface_property = static_cast<FInterfaceProperty*>(property_desc.property_);

        bool success = false;
        FString fstring;
        success = json_value.Get()->TryGetString(fstring);
        SP_ASSERT(success);

        UObject* uobject = Std::toPtrFromString<UObject>(Unreal::toStdString(fstring));

        FScriptInterface* script_interface = interface_property->GetPropertyValuePtr(property_desc.value_ptr_);
        SP_ASSERT(script_interface);
        script_interface->SetObject(uobject);

    } else {

        SP_LOG(Unreal::toStdString(property_desc.property_->GetName()), " is an unsupported type: ", Unreal::toStdString(property_desc.property_->GetClass()->GetName()));
        SP_ASSERT(false);
    }
}

//
// Get and set actor and component stable names
//

bool UnrealUtils::hasStableName(const AActor* actor)
{
    SP_ASSERT(actor);

    std::vector<USpStableNameComponent*> sp_stable_name_components = getComponentsByType<USpStableNameComponent>(actor);
    if (sp_stable_name_components.size() == 1) {
        return true;
    }

    std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());
    if (sp_stable_name_managers.size() == 1) {
        ASpStableNameManager* sp_stable_name_manager = Std::at(sp_stable_name_managers, 0);
        SP_ASSERT(sp_stable_name_manager);
        if (sp_stable_name_manager->hasActor(actor)) {
            return true;
        }
    }

    return false;
}

bool UnrealUtils::hasStableName(const UActorComponent* component)
{
    SP_ASSERT(component);
    return true;
}

std::string UnrealUtils::tryGetStableName(const AActor* actor)
{
    SP_ASSERT(actor);
    if (hasStableName(actor)) {
        return UnrealUtils::getStableName(actor);
    } else {
        #if WITH_EDITOR // defined in an auto-generated header
            if (!actor->HasAnyFlags(EObjectFlags::RF_Transient)) {
                return ASpStableNameManager::getStableNameEditorOnly(actor);
            }
        #endif
        return Unreal::toStdString(actor->GetName());
    }
}

std::string UnrealUtils::getStableName(const AActor* actor)
{
    SP_ASSERT(actor);

    std::vector<USpStableNameComponent*> sp_stable_name_components = getComponentsByType<USpStableNameComponent>(actor);
    for (auto sp_stable_name_component : sp_stable_name_components) {
        SP_ASSERT(sp_stable_name_component);
        return sp_stable_name_component->getStableName();
    }

    std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());
    for (auto sp_stable_name_manager : sp_stable_name_managers) {
        SP_ASSERT(sp_stable_name_manager);
        if (sp_stable_name_manager->hasActor(actor)) {
            return sp_stable_name_manager->getStableName(actor);
        }
    }

    SP_ASSERT(false);
    return "";
}

std::string UnrealUtils::getStableName(const UActorComponent* component, bool include_actor_name, bool actor_must_have_stable_name)
{
    SP_ASSERT(component);

    std::string component_name;

    const USceneComponent* scene_component = Cast<USceneComponent>(component); // no RTTI available, so use Cast instead of dynamic_cast
    if (scene_component) {
        TArray<USceneComponent*> parents;
        scene_component->GetParentComponents(parents);
        for (auto parent : parents) {
            component_name = Unreal::toStdString(parent->GetName()) + "." + component_name;
        }
    }

    component_name = component_name + Unreal::toStdString(component->GetName());

    if (include_actor_name) {
        const AActor* actor = component->GetOwner();
        SP_ASSERT(actor);
        if (actor_must_have_stable_name) {
            component_name = getStableName(actor) + ":" + component_name;
        } else {
            component_name = tryGetStableName(actor) + ":" + component_name;
        }
    }

    return component_name;
}

void UnrealUtils::setStableName(const AActor* actor, const std::string& stable_name)
{
    SP_ASSERT(actor);

    std::vector<USpStableNameComponent*> sp_stable_name_components = getComponentsByType<USpStableNameComponent>(actor);
    for (auto sp_stable_name_component : sp_stable_name_components) {
        SP_ASSERT(sp_stable_name_component);
        sp_stable_name_component->setStableName(stable_name);
    }

    std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());
    for (auto sp_stable_name_manager : sp_stable_name_managers) {
        SP_ASSERT(sp_stable_name_manager);
        if (sp_stable_name_manager->hasActor(actor)) {
            sp_stable_name_manager->setStableName(actor, stable_name);
        } else {
            sp_stable_name_manager->addActor(actor, stable_name); // this case can happen, e.g., for a default pawn with a stable name component
        }
    }
}

#if WITH_EDITOR // defined in an auto-generated header
    void UnrealUtils::requestAddStableNameActor(AActor* actor)
    {
        SP_ASSERT(actor);
        std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());
        for (auto sp_stable_name_manager : sp_stable_name_managers) {
            SP_ASSERT(sp_stable_name_manager);
            sp_stable_name_manager->requestAddActor(actor);
        }
    }

    void UnrealUtils::requestRemoveStableNameActor(AActor* actor)
    {
        SP_ASSERT(actor);
        std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());
        for (auto sp_stable_name_manager : sp_stable_name_managers) {
            SP_ASSERT(sp_stable_name_manager);
            sp_stable_name_manager->requestRemoveActor(actor);
        }
    }

    void UnrealUtils::requestUpdateStableName(AActor* actor)
    {
        SP_ASSERT(actor);

        bool include_from_child_actors = false;
        std::vector<USpStableNameComponent*> sp_stable_name_components = getComponentsByType<USpStableNameComponent>(actor, include_from_child_actors);
        for (auto sp_stable_name_component : sp_stable_name_components) {
            SP_ASSERT(sp_stable_name_component);
            sp_stable_name_component->requestUpdate();
        }

        std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());
        for (auto sp_stable_name_manager : sp_stable_name_managers) {
            SP_ASSERT(sp_stable_name_manager);
            sp_stable_name_manager->requestUpdateActor(actor);
        }
    }

    void UnrealUtils::requestUpdateAllStableNameActors(const UWorld* world)
    {
        SP_ASSERT(world);
        std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(world);
        for (auto sp_stable_name_manager : sp_stable_name_managers) {
            SP_ASSERT(sp_stable_name_manager);
            sp_stable_name_manager->requestUpdateAllActors();
        }
    }
#endif

//
// Find objects
//

std::vector<UObject*> UnrealUtils::findObjects()
{
    return findObjectsByType<UObject>();
}

//
// Helper functions for formatting container properties as strings in the same style as Unreal
//

std::string UnrealUtils::getArrayPropertyValueAsFormattedString(const FProperty* property, const std::vector<std::string>& strings)
{
    SP_ASSERT(property);

    int num_elements = strings.size();
    std::string formatted_string;

    // If the array is empty, then set the join string to be an empty string. Otherwise, if the inner property
    // is a primitive type, then format the entire array on a single line. Otherwise put each element on a new
    // line.

    std::string join_string;
    if (num_elements > 0) {
        if (property->IsA(FStructProperty::StaticClass())) {
            join_string = "\n";
        } else {
            join_string = " ";
        }
    } else {
        join_string = "";
    }

    std::string quote_string = getQuoteStringForProperty(property);

    // Build the formatted string except for indentation.
    formatted_string += "[" + join_string;
    for (int i = 0; i < num_elements; i++) {
        std::string join_prefix_string;
        if (i < num_elements - 1) {
            join_prefix_string = ",";
        } else {
            join_prefix_string = "";
        }
        formatted_string += quote_string + strings.at(i) + quote_string + join_prefix_string + join_string;
    }
    formatted_string += "]";

    // If the property is a struct type and the array is non-empty, then indent.
    if (property->IsA(FStructProperty::StaticClass()) && num_elements > 0) {
        std::vector<std::string> lines = Std::tokenize(formatted_string, "\n");
        SP_ASSERT(lines.size() > 2);
        formatted_string = lines.at(0) + "\n";
        for (int i = 1; i < lines.size() - 1; i++) {
            formatted_string += "\t" + lines.at(i) + "\n";
        }
        formatted_string += lines.at(lines.size() - 1);
    }

    return formatted_string;
}

std::string UnrealUtils::getMapPropertyValueAsFormattedString(const FProperty* key_property, const std::vector<std::string>& key_strings, const FProperty* value_property, const std::vector<std::string>& value_strings)
{
    SP_ASSERT(key_property);
    SP_ASSERT(value_property);
    SP_ASSERT(key_strings.size() == value_strings.size());

    int num_elements = key_strings.size();
    std::string formatted_string;

    // If the map is non-empty, put each element on a new line. Strictly speaking, Unreal appears to always
    // set the join string to a new-line, even when the map is empty. But we don't try to replicate this
    // behavior for better consistency with our array formatting implementation.

    std::string join_string = "\n";
    if (num_elements > 0) {
        join_string = "\n";
    } else {
        join_string = "";
    }

    std::string quote_string = getQuoteStringForProperty(value_property);

    std::string value_prefix_string;
    if (value_property->IsA(FStructProperty::StaticClass())) {
        value_prefix_string = "\n";
    } else {
        value_prefix_string = " ";
    }

    // Build the formatted string except for indentation. Strictly speaking, Unreal appears to add an
    // extra new-line at the start of map strings, but we don't try to replicate this behavior for better
    // consistency with our array formatting implementation.

    formatted_string += "{" + join_string;
    for (int i = 0; i < num_elements; i++) {
        std::string join_prefix_string;
        if (i < num_elements - 1) {
            join_prefix_string = ",";
        } else {
            join_prefix_string = "";
        }
        formatted_string +=
            "\"" + key_strings.at(i) + "\":" + value_prefix_string + quote_string + value_strings.at(i) + quote_string + join_prefix_string + join_string;
    }
    formatted_string += "}";

    // If the map is non-empty, then indent.
    if (num_elements > 0) {
        std::vector<std::string> lines = Std::tokenize(formatted_string, "\n");
        SP_ASSERT(lines.size() > 2);
        formatted_string = lines.at(0) + "\n";
        for (int i = 1; i < lines.size() - 1; i++) {
            formatted_string += "\t" + lines.at(i) + "\n";
        }
        formatted_string += lines.at(lines.size() - 1);
    }

    return formatted_string;
}

std::string UnrealUtils::getQuoteStringForProperty(const FProperty* property)
{
    // If our property is a {bool, int, float, double}, then we expect an input string to contain the
    // property value formatted as a string with no quotes, and we do not need to add any quotes to construct
    // a well-formed JSON string. Likewise, if our property is an {array, set, map, struct}, then we expect
    // an input string to already be a well-formed JSON string, and again we do not need to add any quotes.
    //
    // On the other hand, if our property is a {enum, string, name, object, interface, soft reference}, then
    // we expect an input string to contain a string with no quotes because this is the formatting convention
    // of getPropertyValueFromString(...), and in this case we need to add quotes to construct a well-formed
    // JSON string.
    //
    // If our property is an enum byte, treat it like a string. If it is a non-enum byte, treat it like
    // an int.

    std::string quote_string;

    if (property->IsA(FEnumProperty::StaticClass()) ||
        property->IsA(FStrProperty::StaticClass()) ||
        property->IsA(FNameProperty::StaticClass()) ||
        property->IsA(FObjectProperty::StaticClass()) ||
        property->IsA(FInterfaceProperty::StaticClass()) ||
        property->IsA(FSoftObjectProperty::StaticClass())) {
        quote_string = "\"";
    } else if (property->IsA(FByteProperty::StaticClass())) {
        const FByteProperty* byte_property = static_cast<const FByteProperty*>(property);
        if (byte_property->Enum) {
            quote_string = "\"";
        }
    }

    return quote_string;
}
