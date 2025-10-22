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
#include <Dom/JsonValue.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>              // TCHAR, uint16
#include <Internationalization/Text.h> // FText
#include <JsonObjectConverter.h>
#include <Serialization/JsonReader.h>
#include <Serialization/JsonSerializer.h>
#include <Templates/Casts.h>
#include <Templates/SharedPointer.h>   // TSharedPtr, TSharedRef
#include <UObject/Class.h>             // EIncludeSuperFlag, UClass, UStruct
#include <UObject/NameTypes.h>         // FName
#include <UObject/Object.h>            // UObject
#include <UObject/ObjectMacros.h>      // EPropertyFlags
#include <UObject/UnrealType.h>        // FArrayProperty, FBoolProperty, FByteProperty, FDoubleProperty, FInt8Property, FInt16Property, FInt64Property,
                                       // FFloatProperty, FIntProperty, FMapProperty, FProperty, FScriptArrayHelper, FScriptMapHelper, FScriptSetHelper,
                                       // FSetProperty, FStrProperty, FStructProperty, FUInt16Property, FUInt32Property, FUInt64Property, TFieldIterator

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpSpecialStructs.h"
#include "SpCore/SpStableNameComponent.h"
#include "SpCore/Std.h"

class USceneComponent;
class UWorld;

//
// Find special struct by name. For this function to behave as expected, USpSpecialStructs must have a
// UPROPERTY defined on it named TypeName_ of type TypeName.
//

UStruct* Unreal::findSpecialStructByName(const std::string& struct_name)
{
    // We only need USpSpecialStructs' property metadata here, so we can use the default object.
    UClass* special_structs_static_class = USpSpecialStructs::StaticClass();
    SP_ASSERT(special_structs_static_class);
    UObject* special_structs_default_object = special_structs_static_class->GetDefaultObject();
    SP_ASSERT(special_structs_default_object);
    SpPropertyDesc property_desc = findPropertyByName(special_structs_default_object, struct_name + "_");
    SP_ASSERT(property_desc.property_);
    SP_ASSERT(property_desc.property_->IsA(FStructProperty::StaticClass()));
    FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
    return struct_property->Struct;
}

//
// Get and set object properties, uobject can't be const because we cast it to void*
//

std::string Unreal::getObjectPropertiesAsString(const UObject* uobject)
{
    return getObjectPropertiesAsString(uobject, uobject->GetClass()); // GetClass() returns UClass*, UClass inherits from UStruct
}

std::string Unreal::getObjectPropertiesAsString(const void* value_ptr, const UStruct* ustruct)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);
    FString string;
    FJsonObjectConverter::UStructToJsonObjectString(ustruct, value_ptr, string);
    return toStdString(string);
}

void Unreal::setObjectPropertiesFromString(UObject* uobject, const std::string& string)
{
    SP_ASSERT(uobject);
    return setObjectPropertiesFromString(uobject, uobject->GetClass(), string);
}

void Unreal::setObjectPropertiesFromString(void* value_ptr, const UStruct* ustruct, const std::string& string)
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

//
// Find property by name, get and set property values, uobject can't be const because we cast it to void*,
// value_ptr can't be const because we assign to SpPropertyDesc::value_ptr_.
//

SpPropertyDesc Unreal::findPropertyByName(UObject* uobject, const std::string& property_name)
{
    SP_ASSERT(uobject);
    return findPropertyByName(uobject, uobject->GetClass(), property_name);
}

SpPropertyDesc Unreal::findPropertyByName(void* value_ptr, const UStruct* ustruct, const std::string& property_name)
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

            int index = std::atoi(name_tokens.at(1).c_str());
            SP_ASSERT(index < array_helper.Num());
            property_desc.value_ptr_ = array_property->GetValueAddressAtIndex_Direct(property_desc.property_, property_desc.value_ptr_, index);
            SP_ASSERT(property_desc.value_ptr_);

        } else if (property_desc.property_->IsA(FMapProperty::StaticClass()) && name_tokens.size() == 2) {

            FMapProperty* map_property = static_cast<FMapProperty*>(property_desc.property_);
            FScriptMapHelper map_helper(map_property, property_desc.value_ptr_);

            property_desc.property_ = map_property->ValueProp;
            SP_ASSERT(property_desc.property_);

            bool found = false;
            for (int j = 0; j < map_helper.Num(); j++) {
                SpPropertyDesc inner_key_property_desc;
                inner_key_property_desc.property_ = map_property->KeyProp;
                inner_key_property_desc.value_ptr_ = map_property->GetValueAddressAtIndex_Direct(map_property->KeyProp, property_desc.value_ptr_, j);
                SP_ASSERT(inner_key_property_desc.value_ptr_);
                std::string inner_key_string = getPropertyValueAsString(inner_key_property_desc);

                // If the key type is a string, then we expect the index string to be enclosed in quotes,
                // otherwise we expect it not to be enclosed in quotes.
                if (inner_key_property_desc.property_->IsA(FBoolProperty::StaticClass())  ||
                    inner_key_property_desc.property_->IsA(FIntProperty::StaticClass())   ||
                    inner_key_property_desc.property_->IsA(FByteProperty::StaticClass())  ||
                    inner_key_property_desc.property_->IsA(FEnumProperty::StaticClass())) {
                    if (name_tokens.at(1) == inner_key_string) {
                        found = true;
                    }
                } else if (inner_key_property_desc.property_->IsA(FStrProperty::StaticClass())) {
                    if (name_tokens.at(1) == "\"" + inner_key_string + "\"") {
                        found = true;
                    }
                } else {
                    SP_LOG(name, " has an unsupported key type: ", toStdString(map_property->KeyProp->GetClass()->GetName()));
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
                SP_LOG(name, " is an unsupported type: ", toStdString(property_desc.property_->GetClass()->GetName()));
                SP_ASSERT(false);
            }
        }
    }

    return property_desc;
}

std::string Unreal::getPropertyValueAsString(const SpPropertyDesc& property_desc)
{
    SP_ASSERT(property_desc.property_);
    SP_ASSERT(property_desc.value_ptr_);

    std::string string;

    // We need a special case for FStructProperty in this code block, but not in setPropertyValueFromString(...)
    // because we rely on FJsonValue::TryGetString(...) here, and this function does not work for structs. On
    // the other hand, the equivalent code block in setPropertyValueFromString(...) doesn't need to call
    // TryGetString(), so it also behaves correctly for structs.

    // See Engine/Source/Editor/Kismet/Internal/Reflection/FunctionUtilsPrivate.h for an exaustive
    // enumeration of all possible property types. For now, we only handle the property types that have come
    // up in practice in our experiments.

    if (property_desc.property_->IsA(FBoolProperty::StaticClass())   ||
        property_desc.property_->IsA(FIntProperty::StaticClass())    ||
        property_desc.property_->IsA(FInt8Property::StaticClass())   ||
        property_desc.property_->IsA(FInt16Property::StaticClass())  ||
        // no FInt32Property because the int32 C++ type maps to FIntProperty
        property_desc.property_->IsA(FInt64Property::StaticClass())  ||
        // no FUInt8Property because the uint8 C++ type maps to FByteProperty
        property_desc.property_->IsA(FUInt16Property::StaticClass()) ||
        property_desc.property_->IsA(FUInt32Property::StaticClass()) ||
        property_desc.property_->IsA(FUInt64Property::StaticClass()) ||
        property_desc.property_->IsA(FFloatProperty::StaticClass())  ||
        property_desc.property_->IsA(FDoubleProperty::StaticClass()) ||
        property_desc.property_->IsA(FStrProperty::StaticClass())    ||
        property_desc.property_->IsA(FNameProperty::StaticClass())   ||
        property_desc.property_->IsA(FByteProperty::StaticClass())   ||
        property_desc.property_->IsA(FEnumProperty::StaticClass())) {

        TSharedPtr<FJsonValue> json_value = FJsonObjectConverter::UPropertyToJsonValue(property_desc.property_, property_desc.value_ptr_);
        SP_ASSERT(json_value.IsValid());

        bool success = false;
        FString fstring;
        success = json_value.Get()->TryGetString(fstring);
        SP_ASSERT(success);
        string = toStdString(fstring);

    }  else if (property_desc.property_->IsA(FArrayProperty::StaticClass())) {

        FArrayProperty* array_property = static_cast<FArrayProperty*>(property_desc.property_);
        FScriptArrayHelper array_helper(array_property, property_desc.value_ptr_);

        FProperty* inner_property = array_property->Inner;
        SP_ASSERT(inner_property);

        std::vector<std::string> inner_strings;
        for (int i = 0; i < array_helper.Num(); i++) {
            SpPropertyDesc inner_property_desc;
            inner_property_desc.property_ = inner_property;
            inner_property_desc.value_ptr_ = array_property->GetValueAddressAtIndex_Direct(inner_property, property_desc.value_ptr_, i);
            SP_ASSERT(inner_property_desc.value_ptr_);
            std::string inner_string = getPropertyValueAsString(inner_property_desc);
            inner_strings.push_back(inner_string);
        }

        string = getArrayPropertyValueAsFormattedString(inner_property, inner_strings);

    }  else if (property_desc.property_->IsA(FSetProperty::StaticClass())) {

        FSetProperty* set_property = static_cast<FSetProperty*>(property_desc.property_);
        FScriptSetHelper set_helper(set_property, property_desc.value_ptr_);

        FProperty* inner_property = set_property->ElementProp;
        SP_ASSERT(inner_property);

        std::vector<std::string> inner_strings;
        for (int i = 0; i < set_helper.Num(); i++) {
            SpPropertyDesc inner_property_desc;
            inner_property_desc.property_ = inner_property;
            inner_property_desc.value_ptr_ = set_property->GetValueAddressAtIndex_Direct(inner_property, property_desc.value_ptr_, i);
            SP_ASSERT(inner_property_desc.value_ptr_);
            std::string inner_string = getPropertyValueAsString(inner_property_desc);
            inner_strings.push_back(inner_string);
        }

        string = getArrayPropertyValueAsFormattedString(inner_property, inner_strings);

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
            inner_key_property_desc.value_ptr_ = map_property->GetValueAddressAtIndex_Direct(inner_key_property, property_desc.value_ptr_, i);
            SP_ASSERT(inner_key_property_desc.value_ptr_);
            std::string inner_key_string = getPropertyValueAsString(inner_key_property_desc);
            inner_key_strings.push_back(inner_key_string);

            SpPropertyDesc inner_value_property_desc;
            inner_value_property_desc.property_ = inner_value_property;
            inner_value_property_desc.value_ptr_ = map_property->GetValueAddressAtIndex_Direct(inner_value_property, property_desc.value_ptr_, i);
            SP_ASSERT(inner_value_property_desc.value_ptr_);
            std::string inner_value_string = getPropertyValueAsString(inner_value_property_desc);
            inner_value_strings.push_back(inner_value_string);
        }

        string = getMapPropertyValueAsFormattedString(inner_key_property, inner_key_strings, inner_value_property, inner_value_strings);

    } else if (property_desc.property_->IsA(FStructProperty::StaticClass())) {

        FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
        UStruct* ustruct = struct_property->Struct;
        FString fstring;
        FJsonObjectConverter::UStructToJsonObjectString(ustruct, property_desc.value_ptr_, fstring);
        string = toStdString(fstring);

    } else if (property_desc.property_->IsA(FObjectProperty::StaticClass())) {

        FObjectProperty* object_property = static_cast<FObjectProperty*>(property_desc.property_);
        UObject* uobject = object_property->GetObjectPropertyValue(property_desc.value_ptr_);
        string = Std::toStringFromPtr(uobject);

    } else if (property_desc.property_->IsA(FInterfaceProperty::StaticClass())) {

        FInterfaceProperty* interface_property = static_cast<FInterfaceProperty*>(property_desc.property_);
        FScriptInterface* script_interface = interface_property->GetPropertyValuePtr(property_desc.value_ptr_);
        SP_ASSERT(script_interface);
        UObject* uobject = script_interface->GetObject();
        string = Std::toStringFromPtr(uobject);

    } else {

        SP_LOG(toStdString(property_desc.property_->GetName()), " is an unsupported type: ", toStdString(property_desc.property_->GetClass()->GetName()));
        SP_ASSERT(false);
    }

    return string;
}

void Unreal::setPropertyValueFromString(const SpPropertyDesc& property_desc, const std::string& string)
{
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.property_);

    std::string quote_string = getQuoteStringForProperty(property_desc.property_);

    bool success = false;

    TSharedRef<TJsonReader<>> json_reader = TJsonReaderFactory<>::Create(toFString("{ \"dummy\": " + quote_string + string + quote_string + " }"));
    TSharedPtr<FJsonObject> json_object;
    success = FJsonSerializer::Deserialize(json_reader, json_object);
    SP_ASSERT(success);
    SP_ASSERT(json_object.IsValid());
    TSharedPtr<FJsonValue> json_value = json_object.Get()->TryGetField(toFString("dummy"));
    SP_ASSERT(json_value.IsValid());

    setPropertyValueFromJsonValue(property_desc, json_value);
}

void Unreal::setPropertyValueFromJsonValue(const SpPropertyDesc& property_desc, TSharedPtr<FJsonValue> json_value)
{
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.property_);
    SP_ASSERT(json_value.IsValid());

    // We don't need a special case for FStructProperty in this code block, but we do in getPropertyValueFromString(...).
    // See the discussion above for details.

    // See Engine/Source/Editor/Kismet/Internal/Reflection/FunctionUtilsPrivate.h for an exaustive
    // enumeration of all possible property types. For now, we only handle the property types that have come
    // up in practice in our experiments.

    if (property_desc.property_->IsA(FBoolProperty::StaticClass())   ||
        property_desc.property_->IsA(FIntProperty::StaticClass())    ||
        property_desc.property_->IsA(FInt8Property::StaticClass())   ||
        property_desc.property_->IsA(FInt16Property::StaticClass())  ||
        // no FInt32Property because the int32 C++ type maps to FIntProperty
        property_desc.property_->IsA(FInt64Property::StaticClass())  ||
        // no FUInt8Property because the uint8 C++ type maps to FByteProperty
        property_desc.property_->IsA(FUInt16Property::StaticClass()) ||
        property_desc.property_->IsA(FUInt32Property::StaticClass()) ||
        property_desc.property_->IsA(FUInt64Property::StaticClass()) ||
        property_desc.property_->IsA(FFloatProperty::StaticClass())  ||
        property_desc.property_->IsA(FDoubleProperty::StaticClass()) ||
        property_desc.property_->IsA(FStrProperty::StaticClass())    ||
        property_desc.property_->IsA(FNameProperty::StaticClass())   ||
        property_desc.property_->IsA(FByteProperty::StaticClass())   ||
        property_desc.property_->IsA(FEnumProperty::StaticClass())   ||
        property_desc.property_->IsA(FStructProperty::StaticClass())) {

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
            std::string json_key = toStdString(json_keys[i]);

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

        UObject* uobject = Std::toPtrFromString<UObject>(toStdString(fstring));
        object_property->SetObjectPropertyValue(property_desc.value_ptr_, uobject);

    } else if (property_desc.property_->IsA(FInterfaceProperty::StaticClass())) {

        FInterfaceProperty* interface_property = static_cast<FInterfaceProperty*>(property_desc.property_);

        bool success = false;
        FString fstring;
        success = json_value.Get()->TryGetString(fstring);
        SP_ASSERT(success);

        UObject* uobject = Std::toPtrFromString<UObject>(toStdString(fstring));

        FScriptInterface* script_interface = interface_property->GetPropertyValuePtr(property_desc.value_ptr_);
        SP_ASSERT(script_interface);
        script_interface->SetObject(uobject);

    } else {

        SP_LOG(toStdString(property_desc.property_->GetName()), " is an unsupported type: ", toStdString(property_desc.property_->GetClass()->GetName()));
        SP_ASSERT(false);
    }
}

//
// Find function by name, call function, world can't be const because we cast it to void*, uobject can't be
// const because we call uobject->ProcessEvent(...) which is non-const, ufunction can't be const because we
// call because we pass it to uobject->ProcessEvent(...) which expects non-const
//

UFunction* Unreal::findFunctionByName(const UClass* uclass, const std::string& function_name, EIncludeSuperFlag::Type include_super_flag)
{
    SP_ASSERT(uclass);
    UFunction* function = uclass->FindFunctionByName(toFName(function_name), include_super_flag);
    SP_ASSERT(function);
    return function;
}

std::map<std::string, std::string> Unreal::callFunction(const UWorld* world, UObject* uobject, UFunction* ufunction, const std::map<std::string, std::string>& args, const std::string& world_context)
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
        SpPropertyDesc property_desc;
        property_desc.property_ = *itr;
        SP_ASSERT(property_desc.property_);
        SP_ASSERT(property_desc.property_->HasAnyPropertyFlags(EPropertyFlags::CPF_Parm));

        property_desc.value_ptr_ = property_desc.property_->ContainerPtrToValuePtr<void>(args_vector.data());
        SP_ASSERT(property_desc.value_ptr_);

        std::string property_name = toStdString(property_desc.property_->GetName());

        Std::insert(property_descs, std::move(property_name), std::move(property_desc));
    }

    // Input args must be a subset of the function's args.
    SP_ASSERT(Std::isSubsetOf(Std::keys(args), Std::keys(property_descs)));

    // Set property values.
    for (auto& [property_name, property_desc] : property_descs) {

        // If the current property name has been flagged by the caller as being the special world_context
        // arg, then set the property using the input world pointer instead of using a string in args, and
        // make sure the property name is not also present in args. Strictly speaking, we could assign the
        // world pointer directly to *(property_desc.value_ptr), but this could lead to undefined behavior
        // if the property is not actually a UObject pointer. So we assign via setPropertValueFromString(...),
        // because it provides well-defined behavior in all cases. Either the assignment is possible
        // according to Unreal's JSON assignment rules, in which case setPropertValueFromString(...) will
        // perform the assignment, or the assignment is not possible, in which case setPropertValueFromString(...)
        // will assert.

        if (property_name == world_context) {
            SP_ASSERT(!Std::containsKey(args, property_name));
            setPropertyValueFromString(property_desc, Std::toStringFromPtr(world));
        }

        // If the property name is in args, then set it using the string in args, and make sure the property
        // name was not also flagged by the caller as being the special world_context arg.

        if (Std::containsKey(args, property_name)) {
            SP_ASSERT(property_name != world_context);
            setPropertyValueFromString(property_desc, args.at(property_name));
        }
    }

    // Call function.
    uobject->ProcessEvent(ufunction, args_vector.data());

    // Return all property values because they might have been modified by the function we called.
    std::map<std::string, std::string> return_values;
    for (auto& [property_name, property_desc] : property_descs) {
        Std::insert(return_values, std::move(property_name), getPropertyValueAsString(property_desc));
    }

    return return_values;
}

//
// Find actors unconditionally and return an std::vector or an std::map
//

std::vector<AActor*> Unreal::findActors(const UWorld* world)
{
    return findActorsByType(world);
}

std::map<std::string, AActor*> Unreal::findActorsAsMap(const UWorld* world)
{
    return toMap<AActor>(findActors(world));
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
    return toMap<UActorComponent>(getComponents(actor));
}

// 
// Get children components unconditionally and return an std::vector or an std::map
//

std::vector<USceneComponent*> Unreal::getChildrenComponents(const USceneComponent* parent, bool include_all_descendants)
{
    SP_ASSERT(parent);
    TArray<USceneComponent*> children_tarray;
    parent->GetChildrenComponents(include_all_descendants, children_tarray);
    std::vector<USceneComponent*> children = toStdVector(children_tarray);
    SP_ASSERT(!Std::contains(children, nullptr));
    return children;
}

std::map<std::string, USceneComponent*> Unreal::getChildrenComponentsAsMap(const USceneComponent* parent, bool include_all_descendants)
{
    return toMap<USceneComponent>(getChildrenComponents(parent, include_all_descendants));
}

//
// Get and set actor and component stable names
//

bool Unreal::hasStableName(const AActor* actor)
{
    bool include_from_child_actors = false;
    std::vector<USpStableNameComponent*> sp_stable_name_components = getComponentsByType<USpStableNameComponent>(actor, include_from_child_actors);
    return sp_stable_name_components.size() == 1; // an actor with 0 or multiple USpStableNameComponents is considered to not have a stable name
}

bool Unreal::hasStableName(const UActorComponent* component)
{
    return true;
}

std::string Unreal::tryGetStableName(const AActor* actor)
{
    if (hasStableName(actor)) {
        return Unreal::getStableName(actor);
    } else {
        #if WITH_EDITOR // defined in an auto-generated header
            if (!actor->HasAnyFlags(RF_Transient)) {
                FName folder_path = actor->GetFolderPath();
                if (folder_path.IsNone()) {
                    return Unreal::toStdString(actor->GetActorLabel());
                } else {
                    return Unreal::toStdString(folder_path) + "/" + Unreal::toStdString(actor->GetActorLabel());
                }
            }
        #endif
        return Unreal::toStdString(actor->GetName());
    }
}

std::string Unreal::getStableName(const AActor* actor)
{
    USpStableNameComponent* sp_stable_name_component = getComponentByType<USpStableNameComponent>(actor);
    return toStdString(sp_stable_name_component->StableName);
}

std::string Unreal::getStableName(const UActorComponent* component, bool include_actor_name, bool actor_must_have_stable_name)
{
    SP_ASSERT(component);

    std::string component_name;

    const USceneComponent* scene_component = Cast<USceneComponent>(component); // no RTTI available, so use Cast instead of dynamic_cast
    if (scene_component) {
        TArray<USceneComponent*> parents;
        scene_component->GetParentComponents(parents);
        for (auto parent : parents) {
            component_name = toStdString(parent->GetName()) + "." + component_name;
        }
    }

    component_name = component_name + toStdString(component->GetName());

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

void Unreal::setStableName(const AActor* actor, const std::string& stable_name)
{
    USpStableNameComponent* sp_stable_name_component = getComponentByType<USpStableNameComponent>(actor);
    sp_stable_name_component->StableName = toFString(stable_name);
}

#if WITH_EDITOR // defined in an auto-generated header
    void Unreal::requestUpdateStableName(const AActor* actor)
    {
        bool include_from_child_actors = false;
        std::vector<USpStableNameComponent*> sp_stable_name_components = getComponentsByType<USpStableNameComponent>(actor, include_from_child_actors);
        if (sp_stable_name_components.size() == 1) {
            sp_stable_name_components.at(0)->requestUpdate();
        }
    }
#endif

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

FText Unreal::toFText(const std::string& str)
{
    return FText::FromString(toFString(str));
}

FString Unreal::toFString(const std::string& str)
{
    return FString(UTF8_TO_TCHAR(str.c_str()));
}

//
// Helper functions for formatting container properties as strings in the same style as Unreal
//

std::string Unreal::getArrayPropertyValueAsFormattedString(const FProperty* inner_property, const std::vector<std::string>& inner_strings)
{
    SP_ASSERT(inner_property);

    int num_elements = inner_strings.size();
    std::string formatted_string;

    // If the array is empty, then set the join string to be an empty string. Otherwise, if the inner property
    // is a primitive type, then format the entire array on a single line. Otherwise put each element on a new
    // line.

    std::string join_string;
    if (num_elements > 0) {
        if (inner_property->IsA(FStructProperty::StaticClass())) {
            join_string = "\n";
        } else {
            join_string = " ";
        }
    } else {
        join_string = "";
    }

    std::string quote_string = getQuoteStringForProperty(inner_property);

    // Build the formatted string except for indentation.
    formatted_string += "[" + join_string;
    for (int i = 0; i < num_elements; i++) {
        std::string join_prefix_string;
        if (i < num_elements - 1) {
            join_prefix_string = ",";
        } else {
            join_prefix_string = "";
        }
        formatted_string += quote_string + inner_strings.at(i) + quote_string + join_prefix_string + join_string;
    }
    formatted_string += "]";

    // If the property is a struct type and the array is non-empty, then indent.
    if (inner_property->IsA(FStructProperty::StaticClass()) && num_elements > 0) {
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

std::string Unreal::getMapPropertyValueAsFormattedString(
    const FProperty* inner_key_property, const std::vector<std::string>& inner_key_strings,
    const FProperty* inner_value_property, const std::vector<std::string>& inner_value_strings)
{
    SP_ASSERT(inner_key_property);
    SP_ASSERT(inner_value_property);
    SP_ASSERT(inner_key_strings.size() == inner_value_strings.size());

    int num_elements = inner_key_strings.size();
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

    std::string quote_string = getQuoteStringForProperty(inner_value_property);

    std::string value_prefix_string;
    if (inner_value_property->IsA(FStructProperty::StaticClass())) {
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
            "\"" + inner_key_strings.at(i) + "\":" + value_prefix_string + quote_string + inner_value_strings.at(i) + quote_string + join_prefix_string + join_string;
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

std::string Unreal::getQuoteStringForProperty(const FProperty* property)
{
    // If our property is a {bool, int, float, double}, then we expect an input string to contain the
    // property value formatted as a string with no quotes, and we do not need to add any quotes to construct
    // a well-formed JSON string. Likewise, if our property is an {array, set, map, struct}, then we expect
    // an input string to already be a well-formed JSON string, and again we do not need to add any quotes.
    //
    // On the other hand, if our property is a {enum, string, name, object, interface}, then we expect an
    // input string to contain a string with no quotes because this is the formatting convention of getPropertyValueFromString(...),
    // and in this case we need to add quotes to construct a well-formed JSON string.
    //
    // If our property is an enum byte, treat it like a string. If it is a non-enum byte, treat it like
    // an int.

    std::string quote_string;

    if (property->IsA(FEnumProperty::StaticClass())   ||
        property->IsA(FStrProperty::StaticClass())    ||
        property->IsA(FNameProperty::StaticClass())   ||
        property->IsA(FObjectProperty::StaticClass()) ||
        property->IsA(FInterfaceProperty::StaticClass())) {
        quote_string = "\"";
    } else if (property->IsA(FByteProperty::StaticClass())) {
        const FByteProperty* byte_property = static_cast<const FByteProperty*>(property);
        if (byte_property->Enum) {
            quote_string = "\"";
        }
    }

    return quote_string;
}
