//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/UnrealUtils.h"

#include <stddef.h> // size_t
#include <stdint.h> // uint64_t

#include <bit>          // std::countl_one
#include <charconv>     // std::from_chars
#include <map>
#include <ranges>       // std::views::filter, std::views::transform
#include <string>
#include <system_error> // std::errc
#include <utility>      // std::make_pair, std::pair
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>
#include <Containers/UnrealString.h>        // FString::operator*
#include <CoreGlobals.h>                    // GAllowActorScriptExecutionInEditor
#include <Dom/JsonObject.h>
#include <Dom/JsonValue.h>
#include <Engine/Engine.h>                  // GEngine
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>                   // TCHAR, uint16, uint64
#include <JsonObjectConverter.h>            // EJsonObjectConversionFlags, FJsonObjectConverter
#include <Policies/PrettyJsonPrintPolicy.h> // TPrettyJsonPrintPolicy
#include <Serialization/JsonReader.h>
#include <Serialization/JsonSerializer.h>
#include <Templates/Casts.h>
#include <Templates/SharedPointer.h>        // MakeShared, TSharedPtr, TSharedRef
#include <Templates/UnrealTemplate.h>       // TGuardValue
#include <UObject/Class.h>                  // EIncludeSuperFlag, UClass, UScriptStruct, UStruct
#include <UObject/EnumProperty.h>           // FEnumProperty
#include <UObject/FieldPathProperty.h>      // FFieldPathProperty
#include <UObject/NameTypes.h>              // FName
#include <UObject/Object.h>                 // UObject
#include <UObject/Script.h>                 // EFunctionFlags
#include <UObject/ScriptInterface.h>        // FScriptInterface
#include <UObject/TextProperty.h>           // FTextProperty
#include <UObject/UnrealType.h>             // EFieldIterationFlags, EPropertyPortFlags, FArrayProperty, FBoolProperty, FByteProperty, FDoubleProperty, FInt8Property,
                                            // FInt16Property, FInt64Property, FFloatProperty, FInterfaceProperty, FIntProperty, FLazyObjectProperty, FMapProperty,
                                            // FNameProperty, FNumericProperty, FObjectProperty, FObjectPropertyBase, FProperty, FScriptArrayHelper, FScriptMapHelper,
                                            // FScriptSetHelper, FSetProperty, FSoftObjectProperty, FStrProperty, FStructProperty, FUInt16Property, FUInt32Property,
                                            // FUInt64Property, FWeakObjectProperty, TFieldIterator
#include <UObject/UObjectHash.h>            // GetDerivedClasses
#include <UObject/UObjectIterator.h>        // TObjectIterator

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpStableNameManager.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

class UWorld;

//
// Initialize and terminate
//

void UnrealUtils::initialize()
{
    //
    // Find a property flag that isn't used by any property. If SkipFlags is CPF_None, then
    // FJsonObjectConverter::UStructToJsonObject(...) skips properties that have any of the flags
    // CPF_Deprecated | CPF_Transient, so getObjectPropertiesAsString(...) passes this unused flag if force_no_skip
    // is true to avoid skipping any properties. We combine the flags of every property in every loaded UStruct,
    // and choose the highest bit that isn't set. We choose the highest bit rather than the lowest bit because
    // Unreal assigns new flags in increasing order, and because a rarely used low flag (e.g., CPF_RequiredParm)
    // might not be set on any loaded property. This function is called after engine initialization, so all
    // UStructs from modules in the Default loading phase (or earlier) are loaded.
    //

    uint64 used_property_flags = 0;
    for (TObjectIterator<UStruct> itr; itr; ++itr) {
        for (TFieldIterator<FProperty> property_itr(*itr, EFieldIterationFlags::IncludeDeprecated); property_itr; ++property_itr) {
            used_property_flags |= property_itr->GetPropertyFlags();
        }
    }
    int num_leading_ones = std::countl_one(used_property_flags);
    SP_ASSERT(num_leading_ones < 64);
    s_unused_property_flag_ = static_cast<EPropertyFlags>(uint64{1} << (63 - num_leading_ones));
    SP_ASSERT(s_unused_property_flag_ != CPF_None);
    SP_ASSERT(!(used_property_flags & s_unused_property_flag_));
}

void UnrealUtils::terminate()
{
    s_unused_property_flag_ = CPF_None;
}

//
// Functions for finding static structs
//

std::vector<UScriptStruct*> UnrealUtils::findStaticStructs()
{
    return findStaticStructsByType<UScriptStruct>();
}

std::map<std::string, UScriptStruct*> UnrealUtils::findStaticStructsAsMap(bool include_path)
{
    return toMap(findStaticStructs(), include_path);
}

//
// Functions for getting static classes
//

std::vector<UClass*> UnrealUtils::findStaticClasses()
{
    return findStaticStructsByType<UClass>();
}

std::map<std::string, UClass*> UnrealUtils::findStaticClassesAsMap(bool include_path)
{
    return toMap(findStaticClasses(), include_path);
}

std::vector<UClass*> UnrealUtils::getDerivedClasses(UClass* uclass, bool recursive)
{
    TArray<UClass*> derived_uclasses;
    GetDerivedClasses(uclass, derived_uclasses, recursive);
    return Unreal::toStdVector(derived_uclasses);
}

std::map<std::string, UClass*> UnrealUtils::getDerivedClassesAsMap(UClass* uclass, bool recursive, bool include_path)
{
    return toMap(getDerivedClasses(uclass, recursive), include_path);
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
        std::views::filter([&function_name](auto function) { return Unreal::isNamed(function, function_name); }));
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
    SP_ASSERT(!world->IsPreviewWorld());
    SP_ASSERT(GEngine->GetWorldContextFromWorld(world));

    //
    // If we call any function on a CDO, and the CDO's class is declared with a Within specifier (e.g.,
    // Within=World), and the world is a PIE world, then we will get a benign error message in the editor. To
    // avoid this error message, the solution is to wrap the function call in another UFUNCTION that is defined
    // on a class that isn't declared with a Within specifier.
    //

    if (uobject->HasAnyFlags(RF_ClassDefaultObject) && world->IsEditorWorld() && world->IsGameWorld()) {
        UClass* uclass = uobject->GetClass();
        if (uclass->ClassWithin && uclass->ClassWithin != UObject::StaticClass()) {
            SP_LOG("WARNING: Calling any function on a CDO will generate a benign error message in the editor if: (1) the CDO's class is declared with a Within specifier (e.g., Within=World); and (2) the world is a PIE world. To avoid this error message, wrap the function call in another UFUNCTION that is defined on a class that isn't declared with a Within specifier.");
        }
    }

    // Create buffer to store all args and the return value.
    uint32 num_bytes = ufunction->ParmsSize;
    uint8_t initial_value = 0;
    std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> args_vector(num_bytes, initial_value);
    SP_ASSERT(ufunction->GetMinAlignment() <= 4096);
    SP_ASSERT(4096 % ufunction->GetMinAlignment() == 0);
    SP_ASSERT(Std::isPtrSufficientlyAligned(args_vector.data(), 4096));

    // Explicitly initialize args.
    if (num_bytes > 0) {
        ufunction->InitializeStruct(args_vector.data());
    }

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

            std::string property_name = Unreal::getUnrealName(property_desc.property_);
            Std::insert(property_descs, std::move(property_name), std::move(property_desc));
        }
    }

    // Resolve each arg name to the name of the corresponding property. We use Unreal::isNamed(...) to compare
    // names, so each arg name must match a property name, ignoring case. Std::insert(...) asserts if two args
    // match the same property, and input args must be a subset of the function's args.
    std::map<std::string, std::string> resolved_args;
    for (auto& [arg_name, arg_value] : args) {
        for (auto& [property_name, property_desc] : property_descs) {
            if (Unreal::isNamed(property_desc.property_, arg_name)) {
                Std::insert(resolved_args, property_name, arg_value);
            }
        }
    }
    SP_ASSERT(resolved_args.size() == args.size());

    // In the editor, we perform extra validation to make sure that if the user omits an argument, it will be
    // initialized correctly (i.e., in a way that matches the C++ signature) by InitializeStruct(...). We can
    // only perform this validation in the editor because the necessary metadata (i.e., the default value
    // given in the C++ signature) is not available in standalone builds.

    #if WITH_EDITOR // defined in an auto-generated header
        std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> cpp_default_args_vector(num_bytes, initial_value);
        if (num_bytes > 0) {
            ufunction->InitializeStruct(cpp_default_args_vector.data());
        }
    #endif

    // Set property values.
    for (auto& [property_name, property_desc] : property_descs) {

        // If the current property name has been flagged by the caller as being the special world_context_object
        // arg, then set the property using the input world pointer instead of using a string in args, and
        // make sure the property name is not also present in args. Strictly speaking, we could assign the
        // world pointer directly to *(property_desc.value_ptr), but this could lead to undefined behavior
        // if the property is not actually a UObject pointer. So we assign via setPropertyValueFromString(...),
        // because it provides well-defined behavior in all cases. Either the assignment is possible
        // according to Unreal's JSON assignment rules, in which case setPropertyValueFromString(...) will
        // perform the assignment, or the assignment is not possible, in which case setPropertyValueFromString(...)
        // will assert.

        if (Unreal::isNamed(property_desc.property_, world_context_object)) {
            SP_ASSERT(!Std::containsKey(resolved_args, property_name));
            setPropertyValueFromString(property_desc, toStringFromPtr(world));
        }

        // If the property name is in args, then set it using the string in args, and make sure the property
        // name was not also flagged by the caller as being the special world_context_object arg.

        else if (Std::containsKey(resolved_args, property_name)) {
            SP_ASSERT(!Unreal::isNamed(property_desc.property_, world_context_object));
            setPropertyValueFromString(property_desc, resolved_args.at(property_name));
        }

        // If the caller omitted this parameter (i.e., it is not present in args), the RPC path uses the parameter's
        // default-constructed value, not any C++ default declared in the function signature. Assert that no non-zero
        // default was declared, so the caller is forced to pass the argument explicitly rather than silently getting
        // a value that differs from what the signature implies.

        else {
            #if WITH_EDITOR // defined in an auto-generated header
                FProperty* property = property_desc.property_;
                if (!property->HasAnyPropertyFlags(EPropertyFlags::CPF_ReturnParm)) {
                    std::string cpp_default_key = "CPP_Default_" + Unreal::toStdString(property->GetName()); // passed to UE, so use GetName() instead of Unreal::getUnrealName(...)
                    if (ufunction->HasMetaData(Unreal::toTCharPtr(cpp_default_key))) {
                        std::string cpp_default_value_string = Unreal::toStdString(ufunction->GetMetaData(Unreal::toTCharPtr(cpp_default_key)));
                        void* cpp_default_value_ptr = property->ContainerPtrToValuePtr<void>(cpp_default_args_vector.data());
                        const TCHAR* success = property->ImportText_Direct(Unreal::toTCharPtr(cpp_default_value_string), cpp_default_value_ptr, nullptr, EPropertyPortFlags::PPF_None);
                        SP_ASSERT(success);
                        if (!property->Identical(property_desc.value_ptr_, cpp_default_value_ptr, EPropertyPortFlags::PPF_None)) {
                            SP_LOG("ERROR: UFUNCTION \"", Unreal::getUnrealName(ufunction), "\" argument \"", property_name, "\" is declared in C++ with a non-zero default value of ", cpp_default_value_string, " that is not available in standalone builds, and must therefore be passed in explicitly.");
                            SP_ASSERT(false);
                        }
                    }
                }
            #endif
        }
    }

    #if WITH_EDITOR // defined in an auto-generated header
        if (num_bytes > 0) {
            ufunction->DestroyStruct(cpp_default_args_vector.data());
        }
    #endif

    // Call function. We need to set GAllowActorScriptExecutionInEditor to true because AActor::ProcessEvent
    // blocks calls on non-CDO actor instances in the editor unless this flag is set or the function has
    // CallInEditor metadata.

    {
        TGuardValue<bool> guard_value(GAllowActorScriptExecutionInEditor, true);
        uobject->ProcessEvent(ufunction, args_vector.data());
    }

    // Return all property values because they might have been modified by the function we called.
    std::map<std::string, SpPropertyValue> return_values;
    for (auto& [property_name, property_desc] : property_descs) {
        Std::insert(return_values, property_name, getPropertyValueAsString(property_desc));
    }

    // Explicitly destroy args.
    if (num_bytes > 0) {
        ufunction->DestroyStruct(args_vector.data());
    }

    return return_values;
}

//
// Find properties
//

std::vector<FProperty*> UnrealUtils::findProperties(const UStruct* ustruct, EFieldIterationFlags field_iteration_flags)
{
    SP_ASSERT(ustruct);
    std::vector<FProperty*> properties;
    for (TFieldIterator<FProperty> itr(ustruct, field_iteration_flags); itr; ++itr) {
        FProperty* property = *itr;
        properties.push_back(property);
    }
    return properties;
}

std::map<std::string, FProperty*> UnrealUtils::findPropertiesAsMap(const UStruct* ustruct, EFieldIterationFlags field_iteration_flags)
{
    return toMap(findProperties(ustruct, field_iteration_flags));
}

std::vector<FProperty*> UnrealUtils::findPropertiesByName(const UStruct* ustruct, const std::string& property_name, EFieldIterationFlags field_iteration_flags)
{
    return Std::toVector<FProperty*>(
        findProperties(ustruct, field_iteration_flags) |
        std::views::filter([&property_name](auto property) { return Unreal::isNamed(property, property_name); }));
}

std::vector<FProperty*> UnrealUtils::findPropertiesByFlagsAny(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags)
{
    return Std::toVector<FProperty*>(
        findProperties(ustruct, field_iteration_flags) |
        std::views::filter([property_flags](auto property) { return property->HasAnyPropertyFlags(property_flags); }));
}

std::vector<FProperty*> UnrealUtils::findPropertiesByFlagsAll(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags)
{
    return Std::toVector<FProperty*>(
        findProperties(ustruct, field_iteration_flags) |
        std::views::filter([property_flags](auto property) { return property->HasAllPropertyFlags(property_flags); }));
}

std::map<std::string, FProperty*> UnrealUtils::findPropertiesByNameAsMap(const UStruct* ustruct, const std::string& property_name, EFieldIterationFlags field_iteration_flags)
{
    return toMap(findPropertiesByName(ustruct, property_name, field_iteration_flags));
}

std::map<std::string, FProperty*> UnrealUtils::findPropertiesByFlagsAnyAsMap(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags)
{
    return toMap(findPropertiesByFlagsAny(ustruct, property_flags, field_iteration_flags));
}

std::map<std::string, FProperty*> UnrealUtils::findPropertiesByFlagsAllAsMap(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags)
{
    return toMap(findPropertiesByFlagsAll(ustruct, property_flags, field_iteration_flags));
}

FProperty* UnrealUtils::findPropertyByName(const UStruct* ustruct, const std::string& property_name, EFieldIterationFlags field_iteration_flags)
{
    return toItem(findPropertiesByName(ustruct, property_name, field_iteration_flags));
}

FProperty* UnrealUtils::findPropertyByFlagsAny(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags)
{
    return toItem(findPropertiesByFlagsAny(ustruct, property_flags, field_iteration_flags));
}

FProperty* UnrealUtils::findPropertyByFlagsAll(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags)
{
    return toItem(findPropertiesByFlagsAll(ustruct, property_flags, field_iteration_flags));
}

//
// Get and set object properties, uobject can't be const because we cast it to void*
//

std::string UnrealUtils::getObjectPropertiesAsString(const UObject* uobject, EPropertyFlags check_property_flags, EPropertyFlags skip_property_flags, bool force_no_skip)
{
    return getObjectPropertiesAsString(uobject, uobject->GetClass(), check_property_flags, skip_property_flags, force_no_skip);
}

std::string UnrealUtils::getObjectPropertiesAsString(const void* value_ptr, const UStruct* ustruct, EPropertyFlags check_property_flags, EPropertyFlags skip_property_flags, bool force_no_skip)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);

    // By default, FJsonObjectConverter lowercases the first character of each property name and replaces each
    // occurrence of "ID" with "Id" (e.g., "SegmentationID" becomes "segmentationId"). We skip this step so
    // property names in the output string match the names that appear in C++, because we think the exact C++
    // names are clearer, and make it easier to search for properties online and in the UE codebase. However, we
    // can't strictly guarantee that every property name matches the name that appears in C++, because FNames
    // compare case-insensitively, and outside the editor, the spelling of an FName is the first spelling that was
    // registered in the process (e.g., a property declared as "Result" is returned as "result" if some other code
    // registered FName("result") first; see the name helpers in Unreal.h). To account for this edge case, our
    // Python code uses case-insensitive key lookups throughout.

    // If SkipFlags is CPF_None, then FJsonObjectConverter::UStructToJsonObject(...) skips properties that have any
    // of the flags CPF_Deprecated | CPF_Transient (e.g., AActor::Children). So to avoid skipping any properties if
    // force_no_skip is true, we pass a nonzero flag that isn't used by any property (see initialize()).

    EPropertyFlags json_skip_property_flags = CPF_None;
    if (force_no_skip) {
        SP_ASSERT(s_unused_property_flag_ != CPF_None);
        json_skip_property_flags = s_unused_property_flag_;
    } else {
        json_skip_property_flags = skip_property_flags;
    }

    FString string;
    FJsonObjectConverter::UStructToFormattedJsonObjectString<TCHAR, TPrettyJsonPrintPolicy>(
        ustruct,                                          // StructDefinition
        value_ptr,                                        // Struct
        string,                                           // OutJsonString
        check_property_flags,                             // CheckFlags      (default value: 0)
        json_skip_property_flags,                         // SkipFlags       (default value: 0)
        0,                                                // Indent          (default value: 0)
        nullptr,                                          // ExportCb        (default value: nullptr)
        EJsonObjectConversionFlags::SkipStandardizeCase); // ConversionFlags (default value: EJsonObjectConversionFlags::None)
    return Unreal::toStdString(string);
}

void UnrealUtils::setObjectPropertiesFromString(UObject* uobject, const std::string& string, bool notify_editor, EPropertyFlags check_property_flags, EPropertyFlags skip_property_flags, bool force_no_skip)
{
    SP_ASSERT(uobject);

    UObject* notify_object = nullptr;
    if (notify_editor) {
        notify_object = uobject;
    }

    setObjectPropertiesFromString(static_cast<void*>(uobject), uobject->GetClass(), string, notify_object, check_property_flags, skip_property_flags, force_no_skip);
}

void UnrealUtils::setObjectPropertiesFromString(void* value_ptr, const UStruct* ustruct, const std::string& string, UObject* notify_object, EPropertyFlags check_property_flags, EPropertyFlags skip_property_flags, bool force_no_skip)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);

    // FJsonObjectConverter::JsonObjectToUStruct(...) can modify any number of properties, so we send a
    // whole-object edit-change notification by passing nullptr to PreEditChange(...) and by calling
    // PostEditChange(), which calls PostEditChangeProperty(...) with an FPropertyChangedEvent whose property is
    // nullptr. This is in contrast to setPropertyValueFromString(...), which sends a notification for a specific
    // property chain.

    #if WITH_EDITOR // defined in an auto-generated header
        if (notify_object) {
            notify_object->PreEditChange(nullptr);
        }
    #endif

    bool success = false;
    TSharedRef<TJsonReader<>> json_reader = TJsonReaderFactory<>::Create(Unreal::toFString(string));
    TSharedPtr<FJsonObject> json_object;
    success = FJsonSerializer::Deserialize(json_reader, json_object);
    SP_ASSERT(success);
    SP_ASSERT(json_object.IsValid());

    // Unlike FJsonObjectConverter::UStructToJsonObject(...), FJsonObjectConverter::JsonObjectToUStruct(...)
    // doesn't skip any properties if SkipFlags is CPF_None. So to avoid skipping any properties if force_no_skip is
    // true, we pass CPF_None.

    EPropertyFlags json_skip_property_flags = CPF_None;
    if (force_no_skip) {
        json_skip_property_flags = CPF_None;
    } else {
        json_skip_property_flags = skip_property_flags;
    }

    success = FJsonObjectConverter::JsonObjectToUStruct(
        json_object.ToSharedRef(), // JsonObject
        ustruct,                   // StructDefinition
        value_ptr,                 // OutStruct
        check_property_flags,      // CheckFlags (default value: 0)
        json_skip_property_flags); // SkipFlags  (default value: 0)
    SP_ASSERT(success);

    #if WITH_EDITOR // defined in an auto-generated header
        if (notify_object) {
            notify_object->PostEditChange();
        }
    #endif
}

//
// Get and set properties. Note that uobject can't be const because we cast it to (non-const) void*,
// value_ptr can't be const because we assign to SpPropertyDesc::value_ptr_.
//

SpPropertyDesc UnrealUtils::resolveProperty(UObject* uobject, FProperty* property)
{
    SP_ASSERT(uobject);
    return resolveProperty(uobject, uobject->GetClass(), property, uobject);
}

SpPropertyDesc UnrealUtils::resolveProperty(UObject* uobject, const std::string& property_name)
{
    SP_ASSERT(uobject);
    return resolveProperty(uobject, uobject->GetClass(), property_name, uobject);
}

SpPropertyDesc UnrealUtils::resolveProperty(void* value_ptr, const UStruct* ustruct, FProperty* property, UObject* notify_object, const SpPropertyDesc& parent)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);
    SP_ASSERT(property);
    SP_ASSERT(ustruct->IsChildOf(property->GetOwnerStruct())); // property must be a direct member of ustruct

    // notify_object and parent are mutually exclusive

    if (notify_object) {
        SP_ASSERT(parent == SpPropertyDesc());
    }

    if (parent != SpPropertyDesc()) {
        validatePropertyDesc(parent);
        SP_ASSERT(!parent.notify_objects_.empty());
        SP_ASSERT(parent.value_ptr_ == value_ptr);
        SP_ASSERT(parent.notify_member_properties_.size() == parent.notify_objects_.size());
        SP_ASSERT(parent.notify_element_properties_.size() == parent.notify_objects_.size());
        SP_ASSERT(parent.notify_array_indices_.size() == parent.notify_objects_.size());
        SP_ASSERT(parent.notify_map_keys_.size() == parent.notify_objects_.size());
        notify_object = toPtr<UObject>(parent.notify_objects_.back());
    }

    SpPropertyDesc property_desc;
    property_desc.property_ = property;
    property_desc.type_id_ = Unreal::getCppTypeAsString(property_desc.property_);
    SP_ASSERT(property_desc.type_id_ != "");
    property_desc.value_ptr_ = property_desc.property_->ContainerPtrToValuePtr<void>(value_ptr);
    SP_ASSERT(property_desc.value_ptr_);

    property_desc.notify_objects_            = parent.notify_objects_;
    property_desc.notify_member_properties_  = parent.notify_member_properties_;
    property_desc.notify_element_properties_ = parent.notify_element_properties_;
    property_desc.notify_array_indices_      = parent.notify_array_indices_;
    property_desc.notify_map_keys_           = parent.notify_map_keys_;

    property_desc.notify_objects_.push_back(toHandle(notify_object));
    property_desc.notify_member_properties_.push_back(property);
    property_desc.notify_element_properties_.push_back(nullptr);
    property_desc.notify_array_indices_.push_back(-1);
    property_desc.notify_map_keys_.push_back("");

    return property_desc;
}

SpPropertyDesc UnrealUtils::resolveProperty(void* value_ptr, const UStruct* ustruct, const std::string& property_name, UObject* notify_object, const SpPropertyDesc& parent)
{
    SP_ASSERT(value_ptr);
    SP_ASSERT(ustruct);

    // notify_object and parent are mutually exclusive

    if (notify_object) {
        SP_ASSERT(parent == SpPropertyDesc());
    }

    if (parent != SpPropertyDesc()) {
        validatePropertyDesc(parent);
        SP_ASSERT(!parent.notify_objects_.empty());
        SP_ASSERT(parent.value_ptr_ == value_ptr);
        SP_ASSERT(parent.notify_member_properties_.size() == parent.notify_objects_.size());
        SP_ASSERT(parent.notify_element_properties_.size() == parent.notify_objects_.size());
        SP_ASSERT(parent.notify_array_indices_.size() == parent.notify_objects_.size());
        SP_ASSERT(parent.notify_map_keys_.size() == parent.notify_objects_.size());
        notify_object = toPtr<UObject>(parent.notify_objects_.back());
    }

    std::vector<std::pair<std::string, std::string>> path_elements = parsePropertyPath(property_name);

    SpPropertyDesc property_desc;
    property_desc.value_ptr_ = value_ptr;

    property_desc.notify_objects_            = parent.notify_objects_;
    property_desc.notify_member_properties_  = parent.notify_member_properties_;
    property_desc.notify_element_properties_ = parent.notify_element_properties_;
    property_desc.notify_array_indices_      = parent.notify_array_indices_;
    property_desc.notify_map_keys_           = parent.notify_map_keys_;

    UObject* current_object = notify_object; // base object for the current step; changes at each object traversal

    for (int i = 0; i < path_elements.size(); i++) {

        const auto& [name, index_string] = path_elements.at(i);

        // By default, findPropertyByName(...) includes super class properties and deprecated properties, which matches
        // the lookup behavior of ustruct->FindPropertyByName(...).
        property_desc.property_ = findPropertyByName(ustruct, name);
        SP_ASSERT(property_desc.property_);
        property_desc.type_id_ = Unreal::getCppTypeAsString(property_desc.property_);
        SP_ASSERT(property_desc.type_id_ != "");
        property_desc.value_ptr_ = property_desc.property_->ContainerPtrToValuePtr<void>(property_desc.value_ptr_);
        SP_ASSERT(property_desc.value_ptr_);

        // Record one parallel notify entry per step. notify_objects_ and notify_member_properties_ are set
        // here; notify_member_properties_ is set before the array/map block below so it refers to the
        // container property rather than the array/map element. The array/map/default blocks below set the
        // other three arrays.
        property_desc.notify_objects_.push_back(toHandle(current_object));
        property_desc.notify_member_properties_.push_back(property_desc.property_);

        // If the current property is an array, map, or fixed-size array property, and the name includes the index
        // operator, then update the current property to refer to the element based on the index. For map
        // properties, we expect an index string that is a JSON string literal enclosed in "" quotes if the key
        // type is quoted in JSON (e.g., FString, FName, enum), and not enclosed in quotes otherwise (e.g.,
        // bool, int). See parsePropertyPath(...) for details.

        // We record this before the block below, because the block below updates property_desc.property_ to
        // refer to the element for array and map properties.
        bool fixed_size_array_property = isFixedSizeArrayProperty(property_desc.property_);

        if (property_desc.property_->IsA(FArrayProperty::StaticClass()) && !index_string.empty()) {

            FArrayProperty* array_property = static_cast<FArrayProperty*>(property_desc.property_);
            FScriptArrayHelper array_helper(array_property, property_desc.value_ptr_);

            property_desc.property_ = array_property->Inner;
            SP_ASSERT(property_desc.property_);
            property_desc.type_id_ = Unreal::getCppTypeAsString(property_desc.property_);
            SP_ASSERT(property_desc.type_id_ != "");

            // Require the index string to be a non-negative decimal integer with no other characters.

            int index = -1;
            auto [ptr, error_code] = std::from_chars(index_string.data(), index_string.data() + index_string.size(), index);
            SP_ASSERT(error_code == std::errc());
            SP_ASSERT(ptr == index_string.data() + index_string.size());
            SP_ASSERT(index >= 0);
            SP_ASSERT(index < array_helper.Num());

            property_desc.value_ptr_ = array_helper.GetRawPtr(index);
            SP_ASSERT(property_desc.value_ptr_);

            property_desc.notify_element_properties_.push_back(array_property->Inner);
            property_desc.notify_array_indices_.push_back(index);
            property_desc.notify_map_keys_.push_back("");

        } else if (property_desc.property_->IsA(FMapProperty::StaticClass()) && !index_string.empty()) {

            FMapProperty* map_property = static_cast<FMapProperty*>(property_desc.property_);
            FScriptMapHelper map_helper(map_property, property_desc.value_ptr_);

            property_desc.property_ = map_property->ValueProp;
            SP_ASSERT(property_desc.property_);
            property_desc.type_id_ = Unreal::getCppTypeAsString(property_desc.property_);
            SP_ASSERT(property_desc.type_id_ != "");

            // If the key type is quoted in JSON, then we expect the index string to be a JSON string literal, so
            // we deserialize it to remove the enclosing quotes and unescape any escaped characters. Otherwise, we
            // expect the index string to be unquoted, so we use it directly. FJsonSerializer::Deserialize(...)
            // can't deserialize a top-level scalar value, so we wrap the index string in a single-element array.

            FProperty* key_property = map_property->KeyProp;
            SP_ASSERT(key_property);
            TSharedPtr<FJsonValue> key_json_value;
            if (getQuoteStringForProperty(key_property).empty()) {
                SP_ASSERT(!index_string.starts_with("\""));
                key_json_value = MakeShared<FJsonValueString>(Unreal::toFString(index_string));
            } else {
                SP_ASSERT(index_string.starts_with("\""));
                bool success = false;
                TArray<TSharedPtr<FJsonValue>> json_values;
                TSharedRef<TJsonReader<>> json_reader = TJsonReaderFactory<>::Create(Unreal::toFString("[" + index_string + "]"));
                success = FJsonSerializer::Deserialize(json_reader, json_values);
                SP_ASSERT(success);
                SP_ASSERT(json_values.Num() == 1);
                key_json_value = json_values[0];
                SP_ASSERT(key_json_value.IsValid());
                SP_ASSERT(key_json_value->Type == EJson::String);
            }

            // Import the key string into temporary storage and find the key in the map, to match how we
            // import map keys in setPropertyValueFromJsonValue(...) and how FMapProperty::ImportText_Internal(...)
            // finds existing keys. FindMapIndexWithKey(...) expects a pointer to a key-value pair, but the key is
            // stored at offset 0 in each pair, so we only need temporary storage for the key.

            int num_bytes = key_property->GetElementSize();
            uint8_t initial_value = 0;
            std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> temp_key_vector(num_bytes, initial_value);
            SP_ASSERT(key_property->GetMinAlignment() <= 4096);
            SP_ASSERT(4096 % key_property->GetMinAlignment() == 0);
            SP_ASSERT(Std::isPtrSufficientlyAligned(temp_key_vector.data(), 4096));
            key_property->InitializeValue(temp_key_vector.data());
            SpPropertyDesc key_property_desc;
            key_property_desc.property_ = key_property;
            key_property_desc.value_ptr_ = temp_key_vector.data();
            setPropertyValueFromJsonValue(key_property_desc, key_json_value);
            int32 index = map_helper.FindMapIndexWithKey(temp_key_vector.data());
            key_property->DestroyValue(temp_key_vector.data());

            if (index == INDEX_NONE) {
                SP_LOG(property_name, " refers to a key that isn't in the map.");
                SP_ASSERT(false);
            }

            property_desc.value_ptr_ = map_helper.GetValuePtr(index);
            SP_ASSERT(property_desc.value_ptr_);

            property_desc.notify_element_properties_.push_back(map_property->ValueProp);
            property_desc.notify_array_indices_.push_back(-1);
            property_desc.notify_map_keys_.push_back(index_string);

        } else if (fixed_size_array_property && !index_string.empty()) {

            // A fixed-size array property (e.g., float Foo[4]) is a single FProperty with ArrayDim > 1, and
            // its elements are stored contiguously. So the element's property is the same as the array's
            // property, and we only need to offset value_ptr_.

            property_desc.type_id_ = Unreal::getCppTypeAsString(property_desc.property_, true); // fixed_size_array_element
            SP_ASSERT(property_desc.type_id_ != "");

            // Require the index string to be a non-negative decimal integer with no other characters.

            int index = -1;
            auto [ptr, error_code] = std::from_chars(index_string.data(), index_string.data() + index_string.size(), index);
            SP_ASSERT(error_code == std::errc());
            SP_ASSERT(ptr == index_string.data() + index_string.size());
            SP_ASSERT(index >= 0);
            SP_ASSERT(index < property_desc.property_->ArrayDim);

            property_desc.value_ptr_ = static_cast<uint8*>(property_desc.value_ptr_) + index * property_desc.property_->GetElementSize();

            property_desc.notify_element_properties_.push_back(property_desc.property_);
            property_desc.notify_array_indices_.push_back(index);
            property_desc.notify_map_keys_.push_back("");

        } else {
            SP_ASSERT(index_string.empty()); // the index operator is only supported for array, map, and fixed-size array properties
            property_desc.notify_element_properties_.push_back(nullptr);
            property_desc.notify_array_indices_.push_back(-1);
            property_desc.notify_map_keys_.push_back("");
        }

        // If the current property name is not the last name in our sequence, then by definition the current
        // property refers to something with named properties (i.e., a struct or an object). In this case,
        // we need to update our ustruct pointer, and potentially our value_ptr if our current property
        // refers to an object. In contrast, if the current property name is the last name in our sequence,
        // then there is nothing more to do, because we have already filled in our SpPropertyDesc object.

        if (i < path_elements.size() - 1) {
            SP_ASSERT(!fixed_size_array_property || !index_string.empty()); // a fixed-size array must be indexed before traversing into one of its elements
            if (property_desc.property_->IsA(FObjectProperty::StaticClass())     ||
                property_desc.property_->IsA(FWeakObjectProperty::StaticClass()) ||
                property_desc.property_->IsA(FLazyObjectProperty::StaticClass())) {
                FObjectPropertyBase* object_property = static_cast<FObjectPropertyBase*>(property_desc.property_); // FObjectProperty, FWeakObjectProperty, and FLazyObjectProperty are-a FObjectPropertyBase
                UObject* uobject = object_property->GetObjectPropertyValue(property_desc.value_ptr_);
                SP_ASSERT(uobject);
                property_desc.value_ptr_ = uobject;
                ustruct = uobject->GetClass();
                current_object = uobject;

            } else if (property_desc.property_->IsA(FStructProperty::StaticClass())) {
                FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
                ustruct = struct_property->Struct;

            } else {
                SP_LOG(name, " is an unsupported type: ", Unreal::getUnrealName(property_desc.property_->GetClass()));
                SP_ASSERT(false);
            }
        }
    }

    return property_desc;
}

SpPropertyDesc UnrealUtils::resolveProperty(const SpPropertyDesc& parent, FProperty* property)
{
    validatePropertyDesc(parent);
    SP_ASSERT(parent.property_);
    SP_ASSERT(parent.property_->IsA(FStructProperty::StaticClass()));
    SP_ASSERT(!parent.notify_objects_.empty());

    const UStruct* ustruct = static_cast<FStructProperty*>(parent.property_)->Struct;
    return resolveProperty(parent.value_ptr_, ustruct, property, nullptr, parent);
}

SpPropertyDesc UnrealUtils::resolveProperty(const SpPropertyDesc& parent, const std::string& property_name)
{
    validatePropertyDesc(parent);
    SP_ASSERT(parent.property_);
    SP_ASSERT(parent.property_->IsA(FStructProperty::StaticClass()));
    SP_ASSERT(!parent.notify_objects_.empty());

    const UStruct* ustruct = static_cast<FStructProperty*>(parent.property_)->Struct;
    return resolveProperty(parent.value_ptr_, ustruct, property_name, nullptr, parent);
}

bool UnrealUtils::supportsEditChangeNotify(const SpPropertyDesc& property_desc)
{
    validatePropertyDesc(property_desc);
    // We can send an edit-change notification directly only if the resolved path stays within one object,
    // has at most one array entry (its index is conveyed via the FPropertyChangedEvent array index map),
    // and has no map entries (a map key can't be conveyed).

    SP_ASSERT(!property_desc.notify_objects_.empty());
    SP_ASSERT(property_desc.property_);

    const SpHandle& notify_object = property_desc.notify_objects_.back();
    bool single_object            = Std::all(property_desc.notify_objects_ | std::views::transform([&notify_object](const auto& object) { return object == notify_object; }));
    int num_array_indices         = Std::countNonzero(property_desc.notify_array_indices_ | std::views::transform([](auto array_index) { return array_index != -1; }));
    int num_map_keys              = Std::countNonzero(property_desc.notify_map_keys_ | std::views::transform([](auto& map_key) { return map_key != ""; }));

    return single_object && num_array_indices <= 1 && num_map_keys == 0;
}

void UnrealUtils::preEditChange(const SpPropertyDesc& property_desc)
{
    #if WITH_EDITOR // defined in an auto-generated header
        SP_ASSERT(supportsEditChangeNotify(property_desc));

        UObject* notify_object = toPtr<UObject>(property_desc.notify_objects_.back());
        SP_ASSERT(notify_object);

        FEditPropertyChain property_chain;
        setEditPropertyChain(property_desc, property_chain);

        notify_object->PreEditChange(property_chain);
    #endif
}

void UnrealUtils::postEditChange(const SpPropertyDesc& property_desc)
{
    #if WITH_EDITOR // defined in an auto-generated header
        SP_ASSERT(supportsEditChangeNotify(property_desc));

        UObject* notify_object = toPtr<UObject>(property_desc.notify_objects_.back());
        SP_ASSERT(notify_object);

        FEditPropertyChain property_chain;
        setEditPropertyChain(property_desc, property_chain);

        TArray<TMap<FString, int32>> array_indices_per_object;
        FPropertyChangedChainEvent property_changed_chain_event = getPropertyChangedChainEvent(property_desc, property_chain, array_indices_per_object);
        notify_object->PostEditChangeChainProperty(property_changed_chain_event);
    #endif
}

void UnrealUtils::setEditPropertyChain(const SpPropertyDesc& property_desc, FEditPropertyChain& property_chain)
{
    #if WITH_EDITOR // defined in an auto-generated header
        // supportsEditChangeNotify guarantees every notify_objects_ entry is the same object, so the chain
        // is simply every member property in order from root to leaf
        for (auto member_property : property_desc.notify_member_properties_) {
            property_chain.AddTail(member_property);
        }

        SP_ASSERT(property_chain.GetHead());
        SP_ASSERT(property_chain.GetTail());
        property_chain.SetActiveMemberPropertyNode(property_chain.GetHead()->GetValue());
        property_chain.SetActivePropertyNode(property_chain.GetTail()->GetValue());
    #endif
}

FPropertyChangedChainEvent UnrealUtils::getPropertyChangedChainEvent(const SpPropertyDesc& property_desc, FEditPropertyChain& property_chain, TArray<TMap<FString, int32>>& array_indices_per_object)
{
    // property_chain must already be populated by setEditPropertyChain. property_chain and array_indices_per_object
    // are caller-owned backing storage: the returned FPropertyChangedChainEvent references property_chain by non-owning
    // reference, and its FPropertyChangedEvent base references array_indices_per_object through a non-owning TArrayView
    // (see SetArrayIndexPerObject), so both must outlive the returned event.

    FPropertyChangedEvent property_changed_event(property_desc.property_);

    #if WITH_EDITOR // defined in an auto-generated header
        // The active member property is the top-level member (Head of the edit property chain), which under the
        // single-object invariant guaranteed by supportsEditChangeNotify is notify_member_properties_.front().
        property_changed_event.SetActiveMemberProperty(property_desc.notify_member_properties_.front());

        // Convey array indices for array entries, keyed by the array property name (matching the editor's
        // convention in GenerateArrayIndexMapToObjectNode). The single-object invariant means every entry belongs
        // to the notify object, so there is exactly one object node and no need to filter by object.
        array_indices_per_object.AddDefaulted();
        for (int i = 0; i < property_desc.notify_member_properties_.size(); i++) {
            int array_index = property_desc.notify_array_indices_.at(i);
            if (array_index != -1) {
                FProperty* member_property = property_desc.notify_member_properties_.at(i);
                SP_ASSERT(member_property);
                array_indices_per_object.Last().Add(member_property->GetName(), array_index); // passed to UE, so use GetName() instead of Unreal::getUnrealName(...)
            }
        }

        if (array_indices_per_object.Last().Num() > 0) {
            property_changed_event.ObjectIteratorIndex = 0;
            property_changed_event.SetArrayIndexPerObject(array_indices_per_object);
        }
    #endif

    return FPropertyChangedChainEvent(property_chain, property_changed_event);
}

SpPropertyValue UnrealUtils::getPropertyValueAsString(const SpPropertyDesc& property_desc, bool allow_element_to_recurse)
{
    validatePropertyDesc(property_desc);
    SP_ASSERT(property_desc.property_);
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.type_id_ != "");

    SpPropertyValue property_value;

    // Set type_id_ from the input desc
    property_value.type_id_ = property_desc.type_id_;

    // We need a special case for FStructProperty in this code block, but not in setPropertyValueFromString(...)
    // because we rely on FJsonValue::TryGetString(...) here, and this function does not work for structs. On
    // the other hand, the equivalent code block in setPropertyValueFromString(...) doesn't need to call
    // TryGetString(), so it also behaves correctly for structs.

    // See Engine/Source/Editor/Kismet/Internal/Reflection/FunctionUtilsPrivate.h for an exhaustive
    // enumeration of all possible property types. For now, we only handle the property types that have come
    // up in practice in our experiments.

    bool fixed_size_array = isFixedSizeArray(property_desc, allow_element_to_recurse);
    bool fixed_size_array_element = isFixedSizeArrayElement(property_desc, allow_element_to_recurse);

    if (fixed_size_array) {

        // A fixed-size array property (e.g., float Foo[4]) is a single FProperty with ArrayDim > 1, and its
        // elements are stored contiguously, so we format each element and then format the elements as an
        // array, to match how FJsonObjectConverter::UPropertyToJsonValue(...) formats fixed-size arrays.

        std::vector<std::string> element_strings;
        for (int i = 0; i < property_desc.property_->ArrayDim; i++) {
            SpPropertyDesc element_property_desc;
            element_property_desc.property_ = property_desc.property_;
            element_property_desc.type_id_ = Unreal::getCppTypeAsString(element_property_desc.property_, true); // fixed_size_array_element
            SP_ASSERT(element_property_desc.type_id_ != "");
            element_property_desc.value_ptr_ = static_cast<uint8*>(property_desc.value_ptr_) + i * property_desc.property_->GetElementSize();
            SP_ASSERT(element_property_desc.value_ptr_);
            std::string element_string = getPropertyValueAsString(element_property_desc, false).value_; // allow_element_to_recurse
            element_strings.push_back(element_string);
        }

        property_value.value_ = getArrayPropertyValueAsFormattedString(property_desc.property_, element_strings);

    } else if (property_desc.property_->IsA(FBoolProperty::StaticClass())       ||
               property_desc.property_->IsA(FIntProperty::StaticClass())        ||
               property_desc.property_->IsA(FInt8Property::StaticClass())       ||
               property_desc.property_->IsA(FInt16Property::StaticClass())      ||
               // no FInt32Property because the int32 C++ type maps to FIntProperty
               // no FInt64Property because it could lose precision when passed through FJsonObjectConverter, which stores integers in a double-backed FJsonValue
               // no FUInt8Property because the uint8 C++ type maps to FByteProperty
               property_desc.property_->IsA(FUInt16Property::StaticClass())     ||
               property_desc.property_->IsA(FUInt32Property::StaticClass())     ||
               // no FUInt64Property because it could lose precision when passed through FJsonObjectConverter, which stores integers in a double-backed FJsonValue
               property_desc.property_->IsA(FFloatProperty::StaticClass())      ||
               property_desc.property_->IsA(FDoubleProperty::StaticClass())     ||
               property_desc.property_->IsA(FStrProperty::StaticClass())        ||
               property_desc.property_->IsA(FNameProperty::StaticClass())       ||
               property_desc.property_->IsA(FTextProperty::StaticClass())       ||
               property_desc.property_->IsA(FByteProperty::StaticClass())       ||
               property_desc.property_->IsA(FEnumProperty::StaticClass())       ||
               property_desc.property_->IsA(FSoftObjectProperty::StaticClass()) || // FSoftClassProperty is-a FSoftObjectProperty
               // no FOptionalProperty because FJsonObjectConverter exports a set TOptional<FString> without quotes or escaping, so it can't be round-tripped
               property_desc.property_->IsA(FFieldPathProperty::StaticClass())) {

        TSharedPtr<FJsonValue> json_value;

        if (fixed_size_array_element) {

            // FJsonObjectConverter::UPropertyToJsonValue(...) always reads ArrayDim elements when ArrayDim > 1,
            // so we can't pass it a pointer to a single element. Instead, we copy our element into the first
            // element of a temporary fixed-size array, convert the entire temporary array, and extract the
            // first element. FProperty::InitializeValue(...) and FProperty::DestroyValue(...) operate on all
            // ArrayDim elements, and FProperty::CopySingleValue(...) copies a single element using the
            // element type's copy semantics.

            int num_bytes = property_desc.property_->ArrayDim * property_desc.property_->GetElementSize();
            uint8_t initial_value = 0;
            std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> temp_value_vector(num_bytes, initial_value);
            SP_ASSERT(property_desc.property_->GetMinAlignment() <= 4096);
            SP_ASSERT(4096 % property_desc.property_->GetMinAlignment() == 0);
            SP_ASSERT(Std::isPtrSufficientlyAligned(temp_value_vector.data(), 4096));
            property_desc.property_->InitializeValue(temp_value_vector.data());
            property_desc.property_->CopySingleValue(temp_value_vector.data(), property_desc.value_ptr_);

            TSharedPtr<FJsonValue> temp_json_value = FJsonObjectConverter::UPropertyToJsonValue(property_desc.property_, temp_value_vector.data());
            SP_ASSERT(temp_json_value.IsValid());

            bool success = false;
            TArray<TSharedPtr<FJsonValue>>* temp_json_values;
            success = temp_json_value->TryGetArray(temp_json_values);
            SP_ASSERT(success);
            SP_ASSERT(temp_json_values);
            SP_ASSERT(temp_json_values->Num() == property_desc.property_->ArrayDim);
            json_value = (*temp_json_values)[0];

            property_desc.property_->DestroyValue(temp_value_vector.data());

        } else {
            json_value = FJsonObjectConverter::UPropertyToJsonValue(property_desc.property_, property_desc.value_ptr_);
        }

        SP_ASSERT(json_value.IsValid());

        bool success = false;
        FString fstring;
        success = json_value.Get()->TryGetString(fstring);
        SP_ASSERT(success);

        property_value.value_ = Unreal::toStdString(fstring);

    } else if (property_desc.property_->IsA(FInt64Property::StaticClass())) {

        // Note that int64 and uint64 can exceed 2^53, so the path above FJsonObjectConverter::UPropertyToJsonValue(...)
        // would lose precision. It stores integers in a double-backed FJsonValue and then stringifies via FString::SanitizeFloat).
        // Instead, we format the raw integer to a decimal string directly. The value is emitted unquoted (see
        // getQuoteStringForProperty(...)), so it is a valid JSON number, and Python's json.loads(...) parses it into
        // an arbitrary-precision int. uint64 must use the unsigned accessor because GetSignedIntPropertyValue(...)
        // would misread values above INT64_MAX as negative.

        FNumericProperty* numeric_property = static_cast<FNumericProperty*>(property_desc.property_); // FInt64Property is-a FNumericProperty
        property_value.value_ = Std::toString(numeric_property->GetSignedIntPropertyValue(property_desc.value_ptr_));

    } else if (property_desc.property_->IsA(FUInt64Property::StaticClass())) {

        FNumericProperty* numeric_property = static_cast<FNumericProperty*>(property_desc.property_); // FUInt64Property is-a FNumericProperty
        property_value.value_ = Std::toString(numeric_property->GetUnsignedIntPropertyValue(property_desc.value_ptr_));

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
            inner_property_desc.value_ptr_ = array_helper.GetRawPtr(i);
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
        for (FScriptSetHelper::FIterator itr(set_helper); itr; ++itr) {
            SpPropertyDesc inner_property_desc;
            inner_property_desc.property_ = inner_property;
            inner_property_desc.type_id_ = Unreal::getCppTypeAsString(inner_property_desc.property_);
            SP_ASSERT(inner_property_desc.type_id_ != "");
            inner_property_desc.value_ptr_ = set_helper.GetElementPtr(itr);
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
        for (FScriptMapHelper::FIterator itr(map_helper); itr; ++itr) {
            SpPropertyDesc inner_key_property_desc;
            inner_key_property_desc.property_ = inner_key_property;
            inner_key_property_desc.type_id_ = Unreal::getCppTypeAsString(inner_key_property_desc.property_);
            SP_ASSERT(inner_key_property_desc.type_id_ != "");
            inner_key_property_desc.value_ptr_ = map_helper.GetKeyPtr(itr);
            SP_ASSERT(inner_key_property_desc.value_ptr_);
            std::string inner_key_string = getPropertyValueAsString(inner_key_property_desc).value_;
            inner_key_strings.push_back(inner_key_string);

            SpPropertyDesc inner_value_property_desc;
            inner_value_property_desc.property_ = inner_value_property;
            inner_value_property_desc.type_id_ = Unreal::getCppTypeAsString(inner_value_property_desc.property_);
            SP_ASSERT(inner_value_property_desc.type_id_ != "");
            inner_value_property_desc.value_ptr_ = map_helper.GetValuePtr(itr);
            SP_ASSERT(inner_value_property_desc.value_ptr_);
            std::string inner_value_string = getPropertyValueAsString(inner_value_property_desc).value_;
            inner_value_strings.push_back(inner_value_string);
        }

        // In most cases, TMap keys are already handled case-insensitively (e.g., when the key type is FString
        // or FName). So most TMaps can't have two keys that differ only by case. An exception to this rule is
        // when the key type is a UENUM, and the enum has two distinct values where the C++ names of those
        // values differ only by case. In this setting, the keys in the TMap are the underlying numerical enum
        // values, so the two keys are distinct. But getPropertyValueAsString(...) converts each key to its
        // enum name, and enum names are stored as FNames, so the two enum names have identical strings. The
        // formatted JSON string would then contain duplicate keys, which json.loads(...) would silently
        // collapse in Python, so we detect this case here.
        if (!Std::allUnique(inner_key_strings)) {
            std::map<std::string, uint64_t> counts = Std::count(inner_key_strings);
            std::vector<std::string> duplicate_key_strings = Std::toVector<std::string>(
                counts |
                std::views::filter([](const auto& pair) { return pair.second > 1; }) |
                std::views::transform([](const auto& pair) { return pair.first; }));
            SP_LOG("    ERROR: TMap property \"", Unreal::getUnrealName(map_property), "\" has distinct keys with identical string representations: [", Std::join(duplicate_key_strings, ", "), "]");
            SP_ASSERT(false);
        }

        property_value.value_ = getMapPropertyValueAsFormattedString(inner_key_property, inner_key_strings, inner_value_property, inner_value_strings);

    } else if (property_desc.property_->IsA(FStructProperty::StaticClass())) {

        FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
        UStruct* ustruct = struct_property->Struct;
        property_value.value_ = getObjectPropertiesAsString(property_desc.value_ptr_, ustruct);

    } else if (property_desc.property_->IsA(FObjectProperty::StaticClass())     ||
               property_desc.property_->IsA(FWeakObjectProperty::StaticClass()) ||
               property_desc.property_->IsA(FLazyObjectProperty::StaticClass())) {

        FObjectPropertyBase* object_property = static_cast<FObjectPropertyBase*>(property_desc.property_); // FObjectProperty, FWeakObjectProperty, and FLazyObjectProperty are-a FObjectPropertyBase
        UObject* uobject = object_property->GetObjectPropertyValue(property_desc.value_ptr_);
        property_value.value_ = toStringFromPtr(uobject);

    } else if (property_desc.property_->IsA(FInterfaceProperty::StaticClass())) {

        FInterfaceProperty* interface_property = static_cast<FInterfaceProperty*>(property_desc.property_);
        FScriptInterface* script_interface = interface_property->GetPropertyValuePtr(property_desc.value_ptr_);
        SP_ASSERT(script_interface);
        UObject* uobject = script_interface->GetObject();
        property_value.value_ = toStringFromPtr(uobject);

    } else {
        SP_LOG("    ERROR: Property ", Unreal::getUnrealName(property_desc.property_), " is an unsupported type: ", Unreal::getUnrealName(property_desc.property_->GetClass()));
        SP_ASSERT(false);
    }

    return property_value;
}

void UnrealUtils::setPropertyValueFromString(const SpPropertyDesc& property_desc, const std::string& string, bool notify_editor)
{
    validatePropertyDesc(property_desc);
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.property_);

    // If our property is an entire fixed-size array, then we expect the input string to already be a
    // well-formed JSON array, so we don't add any quotes or escape any characters.

    bool fixed_size_array = isFixedSizeArray(property_desc, true); // allow_element_to_recurse

    std::string quote_string;
    if (!fixed_size_array) {
        quote_string = getQuoteStringForProperty(property_desc.property_);
    }

    // FString, FName, and FText values can contain arbitrary characters, so we need to escape them to
    // construct a well-formed JSON string. We don't need to escape the other quoted property types, because their values
    // can't contain characters that need escaping: enum values are C++ identifiers, soft object values are
    // object paths, field path values are property paths, and object and interface values are pointer
    // strings. For these types, we assert that no characters need escaping.

    std::string json_string;
    if (fixed_size_array) {
        json_string = string;
    } else if (property_desc.property_->IsA(FStrProperty::StaticClass())  ||
               property_desc.property_->IsA(FNameProperty::StaticClass()) ||
               property_desc.property_->IsA(FTextProperty::StaticClass())) {
        json_string = getJsonString(string);
    } else if (!quote_string.empty()) {
        std::string special_chars = "\"\\\n\r\t\b\f";
        SP_ASSERT(!Std::any(special_chars | std::views::transform([&string](char c) { return Std::contains(string, std::string(1, c)); })));
        json_string = string;
    } else {
        json_string = string;
    }

    bool success = false;

    // Deserialize with StoreNumbersAsStrings so numeric tokens are preserved as exact strings rather than
    // flattened to a double. This lets the FInt64Property/FUInt64Property cases in setPropertyValueFromJsonValue(...)
    // recover the exact value for scalars and for container elements, which are parsed here before those
    // cases run. Other numeric types are unaffected.

    // FJsonSerializer::Deserialize(...) can't deserialize a top-level scalar value, so we wrap our value in a
    // single-element JSON array and extract the element.

    TSharedRef<TJsonReader<>> json_reader = TJsonReaderFactory<>::Create(Unreal::toFString("[" + quote_string + json_string + quote_string + "]"));
    TArray<TSharedPtr<FJsonValue>> json_values;
    success = FJsonSerializer::Deserialize(json_reader, json_values, FJsonSerializer::EFlags::StoreNumbersAsStrings);
    SP_ASSERT(success);
    SP_ASSERT(json_values.Num() == 1);
    TSharedPtr<FJsonValue> json_value = json_values[0];
    SP_ASSERT(json_value.IsValid());

    setPropertyValueFromJsonValue(property_desc, json_value, notify_editor);
}

void UnrealUtils::setPropertyValueFromJsonValue(const SpPropertyDesc& property_desc, TSharedPtr<FJsonValue> json_value, bool notify_editor, bool allow_element_to_recurse)
{
    validatePropertyDesc(property_desc);
    SP_ASSERT(property_desc.value_ptr_);
    SP_ASSERT(property_desc.property_);
    SP_ASSERT(json_value.IsValid());

    #if WITH_EDITOR // defined in an auto-generated header
        if (notify_editor) {
            preEditChange(property_desc);
        }
    #endif

    // We don't need a special case for FStructProperty in this code block, but we do in getPropertyValueAsString(...).
    // See the discussion above for details.

    // See Engine/Source/Editor/Kismet/Internal/Reflection/FunctionUtilsPrivate.h for an exhaustive
    // enumeration of all possible property types. For now, we only handle the property types that have come
    // up in practice in our experiments.

    bool fixed_size_array = isFixedSizeArray(property_desc, allow_element_to_recurse);
    bool fixed_size_array_element = isFixedSizeArrayElement(property_desc, allow_element_to_recurse);

    if (fixed_size_array) {

        // See the corresponding comment in getPropertyValueAsString(...).

        bool success = false;
        TArray<TSharedPtr<FJsonValue>>* json_values;
        success = json_value->TryGetArray(json_values);
        SP_ASSERT(success);
        SP_ASSERT(json_values);
        SP_ASSERT(json_values->Num() == property_desc.property_->ArrayDim);

        for (int i = 0; i < json_values->Num(); i++) {
            TSharedPtr<FJsonValue> element_json_value = (*json_values)[i];

            // Skip null elements, leaving them unchanged, to match FJsonObjectConverter::JsonValueToUProperty(...).
            if (element_json_value.IsValid() && !element_json_value->IsNull()) {
                SpPropertyDesc element_property_desc;
                element_property_desc.property_ = property_desc.property_;
                element_property_desc.value_ptr_ = static_cast<uint8*>(property_desc.value_ptr_) + i * property_desc.property_->GetElementSize();
                SP_ASSERT(element_property_desc.value_ptr_);

                setPropertyValueFromJsonValue(element_property_desc, element_json_value, false, false); // notify_editor, allow_element_to_recurse
            }
        }

    } else if (property_desc.property_->IsA(FBoolProperty::StaticClass())       ||
               property_desc.property_->IsA(FIntProperty::StaticClass())        ||
               property_desc.property_->IsA(FInt8Property::StaticClass())       ||
               property_desc.property_->IsA(FInt16Property::StaticClass())      ||
               // no FInt32Property because the int32 C++ type maps to FIntProperty
               // no FInt64Property because it could lose precision when passed through FJsonObjectConverter, which stores integers in a double-backed FJsonValue
               // no FUInt8Property because the uint8 C++ type maps to FByteProperty
               property_desc.property_->IsA(FUInt16Property::StaticClass())     ||
               property_desc.property_->IsA(FUInt32Property::StaticClass())     ||
               // no FUInt64Property because it could lose precision when passed through FJsonObjectConverter, which stores integers in a double-backed FJsonValue
               property_desc.property_->IsA(FFloatProperty::StaticClass())      ||
               property_desc.property_->IsA(FDoubleProperty::StaticClass())     ||
               property_desc.property_->IsA(FStrProperty::StaticClass())        ||
               property_desc.property_->IsA(FNameProperty::StaticClass())       ||
               property_desc.property_->IsA(FTextProperty::StaticClass())       ||
               property_desc.property_->IsA(FByteProperty::StaticClass())       ||
               property_desc.property_->IsA(FEnumProperty::StaticClass())       ||
               property_desc.property_->IsA(FStructProperty::StaticClass())     ||
               property_desc.property_->IsA(FSoftObjectProperty::StaticClass()) || // FSoftClassProperty is-a FSoftObjectProperty
               // no FOptionalProperty because FJsonObjectConverter exports a set TOptional<FString> without quotes or escaping, so it can't be round-tripped
               property_desc.property_->IsA(FFieldPathProperty::StaticClass())) {

        if (fixed_size_array_element) {

            // See the corresponding comment in getPropertyValueAsString(...). FJsonObjectConverter::JsonValueToUProperty(...)
            // always writes to a fixed-size array when ArrayDim > 1, so we can't pass it a pointer to a single element.
            // Instead, we convert a single-element JSON array into the first element of a temporary fixed-size array, and
            // then copy the first element into our element. We pass a single-element JSON array, rather than a scalar JSON
            // value, to avoid a warning from FJsonObjectConverter::JsonValueToUProperty(...).

            int num_bytes = property_desc.property_->ArrayDim * property_desc.property_->GetElementSize();
            uint8_t initial_value = 0;
            std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> temp_value_vector(num_bytes, initial_value);
            SP_ASSERT(property_desc.property_->GetMinAlignment() <= 4096);
            SP_ASSERT(4096 % property_desc.property_->GetMinAlignment() == 0);
            SP_ASSERT(Std::isPtrSufficientlyAligned(temp_value_vector.data(), 4096));
            property_desc.property_->InitializeValue(temp_value_vector.data());

            TArray<TSharedPtr<FJsonValue>> temp_json_values = {json_value};
            TSharedPtr<FJsonValue> temp_json_value = MakeShared<FJsonValueArray>(temp_json_values);

            bool success = false;
            success = FJsonObjectConverter::JsonValueToUProperty(temp_json_value, property_desc.property_, temp_value_vector.data());
            SP_ASSERT(success);

            property_desc.property_->CopySingleValue(property_desc.value_ptr_, temp_value_vector.data());

            property_desc.property_->DestroyValue(temp_value_vector.data());

        } else {
            bool success = false;
            success = FJsonObjectConverter::JsonValueToUProperty(json_value, property_desc.property_, property_desc.value_ptr_);
            SP_ASSERT(success);
        }

    } else if (property_desc.property_->IsA(FInt64Property::StaticClass())) {

        // See the corresponding comment in getPropertyValueAsString(...). setPropertyValueFromString(...) deserializes
        // with StoreNumbersAsStrings, so json_value is a string-backed number and TryGetString(...) returns the exact
        // decimal digits, which we parse directly rather than casting through a double. Note that uint64 must use the
        // unsigned parse and the unsigned SetIntPropertyValue(...) overload to represent values above INT64_MAX correctly.

        FString fstring;
        bool success = json_value->TryGetString(fstring);
        SP_ASSERT(success);
        FNumericProperty* numeric_property = static_cast<FNumericProperty*>(property_desc.property_); // FInt64Property is-a FNumericProperty
        numeric_property->SetIntPropertyValue(property_desc.value_ptr_, static_cast<int64>(FCString::Strtoi64(*fstring, nullptr, 10)));

    } else if (property_desc.property_->IsA(FUInt64Property::StaticClass())) {

        // See comment for FInt64Property above.

        FString fstring;
        bool success = json_value->TryGetString(fstring);
        SP_ASSERT(success);
        FNumericProperty* numeric_property = static_cast<FNumericProperty*>(property_desc.property_); // FUInt64Property is-a FNumericProperty
        numeric_property->SetIntPropertyValue(property_desc.value_ptr_, static_cast<uint64>(FCString::Strtoui64(*fstring, nullptr, 10)));

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

            // Skip null elements, leaving them default-initialized, to match FJsonObjectConverter::JsonValueToUProperty(...).
            if (inner_json_value.IsValid() && !inner_json_value->IsNull()) {
                SpPropertyDesc inner_property_desc;
                inner_property_desc.property_ = inner_property;
                inner_property_desc.value_ptr_ = array_helper.GetRawPtr(i);
                SP_ASSERT(inner_property_desc.value_ptr_);

                setPropertyValueFromJsonValue(inner_property_desc, inner_json_value);
            }
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

        // Import each element into temporary storage and only add it to the set if it isn't already present,
        // to match FSetProperty::ImportText_Internal(...). FindElementIndex(...) does a linear search that
        // doesn't depend on the hash, so it is safe to call before Rehash().
        int num_bytes = inner_property->GetElementSize();
        uint8_t initial_value = 0;
        std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> temp_element_vector(num_bytes, initial_value);
        SP_ASSERT(inner_property->GetMinAlignment() <= 4096);
        SP_ASSERT(4096 % inner_property->GetMinAlignment() == 0);
        SP_ASSERT(Std::isPtrSufficientlyAligned(temp_element_vector.data(), 4096));

        for (int i = 0; i < json_values->Num(); i++) {
            TSharedPtr<FJsonValue> inner_json_value = (*json_values)[i];

            // Skip null elements to match FJsonObjectConverter::JsonValueToUProperty(...).
            if (inner_json_value.IsValid() && !inner_json_value->IsNull()) {
                inner_property->InitializeValue(temp_element_vector.data());

                SpPropertyDesc inner_property_desc;
                inner_property_desc.property_ = inner_property;
                inner_property_desc.value_ptr_ = temp_element_vector.data();

                setPropertyValueFromJsonValue(inner_property_desc, inner_json_value);

                if (set_helper.FindElementIndex(temp_element_vector.data()) == INDEX_NONE) {
                    int32 index = set_helper.AddDefaultValue_Invalid_NeedsRehash();
                    uint8* element_ptr = set_helper.GetElementPtr(index);
                    SP_ASSERT(element_ptr);
                    inner_property->CopyCompleteValue(element_ptr, temp_element_vector.data());
                }

                inner_property->DestroyValue(temp_element_vector.data());
            }
        }

        set_helper.Rehash();

    }  else if (property_desc.property_->IsA(FMapProperty::StaticClass())) {

        bool success = false;
        TSharedPtr<FJsonObject>* json_object;
        success = json_value->TryGetObject(json_object);
        SP_ASSERT(success);
        SP_ASSERT(json_object->IsValid());

        FMapProperty* map_property = static_cast<FMapProperty*>(property_desc.property_);
        FScriptMapHelper map_helper(map_property, property_desc.value_ptr_);
        map_helper.EmptyValues(json_object->Get()->Values.Num());

        FProperty* inner_key_property = map_property->KeyProp;
        SP_ASSERT(inner_key_property);
        FProperty* inner_value_property = map_property->ValueProp;
        SP_ASSERT(inner_value_property);

        // Import each key into temporary storage and only add the key-value pair to the map if the key isn't
        // already present, e.g., if "1" and "01" are both keys in the input JSON object for an int32 key type, to
        // match FMapProperty::ImportText_Internal(...). FindMapIndexWithKey(...) does a linear search that doesn't
        // depend on the hash, so it is safe to call before Rehash(). The key is stored at offset 0 in each pair, so
        // we only need temporary storage for the key.
        int num_bytes = inner_key_property->GetElementSize();
        uint8_t initial_value = 0;
        std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> temp_key_vector(num_bytes, initial_value);
        SP_ASSERT(inner_key_property->GetMinAlignment() <= 4096);
        SP_ASSERT(4096 % inner_key_property->GetMinAlignment() == 0);
        SP_ASSERT(Std::isPtrSufficientlyAligned(temp_key_vector.data(), 4096));

        for (auto& [json_key, inner_json_value] : json_object->Get()->Values) {

            // Skip null values to match FJsonObjectConverter::JsonValueToUProperty(...).
            if (inner_json_value.IsValid() && !inner_json_value->IsNull()) {
                inner_key_property->InitializeValue(temp_key_vector.data());

                // Wrap the key in an FJsonValueString to match FJsonObjectConverter::JsonValueToUProperty(...).
                SpPropertyDesc inner_key_property_desc;
                inner_key_property_desc.property_ = inner_key_property;
                inner_key_property_desc.value_ptr_ = temp_key_vector.data();

                setPropertyValueFromJsonValue(inner_key_property_desc, MakeShared<FJsonValueString>(json_key));

                if (map_helper.FindMapIndexWithKey(temp_key_vector.data()) == INDEX_NONE) {
                    int32 index = map_helper.AddDefaultValue_Invalid_NeedsRehash();
                    uint8* key_ptr = map_helper.GetKeyPtr(index);
                    SP_ASSERT(key_ptr);
                    inner_key_property->CopyCompleteValue(key_ptr, temp_key_vector.data());

                    SpPropertyDesc inner_value_property_desc;
                    inner_value_property_desc.property_ = inner_value_property;
                    inner_value_property_desc.value_ptr_ = map_helper.GetValuePtr(index);
                    SP_ASSERT(inner_value_property_desc.value_ptr_);

                    setPropertyValueFromJsonValue(inner_value_property_desc, inner_json_value);
                }

                inner_key_property->DestroyValue(temp_key_vector.data());
            }
        }

        map_helper.Rehash();

    } else if (property_desc.property_->IsA(FObjectProperty::StaticClass())     ||
               property_desc.property_->IsA(FWeakObjectProperty::StaticClass()) ||
               property_desc.property_->IsA(FLazyObjectProperty::StaticClass())) {

        FObjectPropertyBase* object_property = static_cast<FObjectPropertyBase*>(property_desc.property_); // FObjectProperty, FWeakObjectProperty, and FLazyObjectProperty are-a FObjectPropertyBase

        bool success = false;
        FString fstring;
        success = json_value.Get()->TryGetString(fstring);
        SP_ASSERT(success);

        UObject* uobject = toPtrFromString<UObject>(Unreal::toStdString(fstring));
        object_property->SetObjectPropertyValue(property_desc.value_ptr_, uobject);

    } else if (property_desc.property_->IsA(FInterfaceProperty::StaticClass())) {

        FInterfaceProperty* interface_property = static_cast<FInterfaceProperty*>(property_desc.property_);

        bool success = false;
        FString fstring;
        success = json_value.Get()->TryGetString(fstring);
        SP_ASSERT(success);

        UObject* uobject = toPtrFromString<UObject>(Unreal::toStdString(fstring));

        FScriptInterface* script_interface = interface_property->GetPropertyValuePtr(property_desc.value_ptr_);
        SP_ASSERT(script_interface);
        script_interface->SetObject(uobject);

    } else {

        SP_LOG(Unreal::getUnrealName(property_desc.property_), " is an unsupported type: ", Unreal::getUnrealName(property_desc.property_->GetClass()));
        SP_ASSERT(false);
    }

    #if WITH_EDITOR // defined in an auto-generated header
        if (notify_editor) {
            postEditChange(property_desc);
        }
    #endif
}

//
// Find objects
//

std::vector<UObject*> UnrealUtils::findObjects()
{
    return findObjectsByType<UObject>();
}

//
// Find actors unconditionally and return an std::vector or an std::map
//

std::vector<AActor*> UnrealUtils::findActors(const UWorld* world)
{
    return findActorsByType(world);
}

std::map<std::string, AActor*> UnrealUtils::findActorsAsMap(const UWorld* world, bool include_unreal_name)
{
    return toMap(findActors(world), include_unreal_name);
}

// 
// Get components unconditionally and return an std::vector or an std::map
//

std::vector<UActorComponent*> UnrealUtils::getComponents(const AActor* actor)
{
    return getComponentsByType(actor);
}

std::map<std::string, UActorComponent*> UnrealUtils::getComponentsAsMap(const AActor* actor, bool include_actor_stable_name, bool include_actor_unreal_name)
{
    return toMap(getComponents(actor), include_actor_stable_name, include_actor_unreal_name);
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

std::map<std::string, USceneComponent*> UnrealUtils::getChildrenComponentsAsMap(const AActor* parent, bool include_all_descendants, bool include_actor_stable_name, bool include_actor_unreal_name)
{
    return toMap(getChildrenComponents(parent, include_all_descendants), include_actor_stable_name, include_actor_unreal_name);
}

std::map<std::string, USceneComponent*> UnrealUtils::getChildrenComponentsAsMap(const USceneComponent* parent, bool include_all_descendants, bool include_actor_stable_name, bool include_actor_unreal_name)
{
    return toMap(getChildrenComponents(parent, include_all_descendants), include_actor_stable_name, include_actor_unreal_name);
}

//
// Get and set actor and component stable names
//

std::string UnrealUtils::getStableName(const AActor* actor, bool include_unreal_name)
{
    SP_ASSERT(actor);

    UWorld* world = actor->GetWorld();
    SP_ASSERT(world);
    SP_ASSERT(!world->IsPreviewWorld());
    SP_ASSERT(GEngine->GetWorldContextFromWorld(world));

    std::string stable_name;

    // If we're in the editor but outside of a PIE session, stable names are given by resolveStableName(...)
    // The only way to set an actor's stable name in this case is to call actor->SetFolderPath(...) and actor->SetActorLabel(...).

    if (world->IsEditorWorld() && !world->IsGameWorld()) {
        #if WITH_EDITOR // defined in an auto-generated header
            stable_name = resolveStableName(actor);
        #else
            SP_ASSERT(false);
        #endif

    // Otherwise, we're either in a PIE game or a standalone game. In this case, we try to go through our
    // stateful interfaces.

    } else {

        // If there are multiple USpStableNameComponents, then it is ambiguous which one we should get the
        // stable name from, so we require there to be at most one.

        std::vector<USpStableNameComponent*> sp_stable_name_components = getComponentsByType<USpStableNameComponent>(actor);
        SP_ASSERT(sp_stable_name_components.size() <= 1);

        // Try to find a UStableNameComponent on the actor.

        if (sp_stable_name_components.size() == 1) {
            USpStableNameComponent* sp_stable_name_component = sp_stable_name_components.at(0);
            SP_ASSERT(sp_stable_name_component);
            stable_name = sp_stable_name_component->getStableName();

        // Otherwise, try various fallbacks.

        } else {

            // If there are multiple ASpStableNameManagers, then it is ambiguous which one we should get the
            // stable name from, so we require there to be at most one.

            std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());
            SP_ASSERT(sp_stable_name_managers.size() <= 1);

            // Try to find an ASpStableNameManager.

            if (sp_stable_name_managers.size() == 1) {
                ASpStableNameManager* sp_stable_name_manager = sp_stable_name_managers.at(0);
                SP_ASSERT(sp_stable_name_manager);

                // If the actor is registered with the stable name manager, then use getStableName(...).

                if (sp_stable_name_manager->hasActor(actor)) {
                    stable_name = sp_stable_name_manager->getStableName(actor);

                // Otherwise, try various fallbacks.

                } else {

                    // If we're in a PIE game, then use resolveStableName(actor),
                    // otherwise use the actor's Unreal name.

                    if (world->IsEditorWorld() && world->IsGameWorld()) {
                        #if WITH_EDITOR // defined in an auto-generated header
                            stable_name = resolveStableName(actor);
                        #else
                            SP_ASSERT(false);
                        #endif
                    } else if (!world->IsEditorWorld() && world->IsGameWorld()) {
                        stable_name = Unreal::getUnrealName(actor);
                    } else {
                        SP_ASSERT(false);
                    }
                }

            // Otherwise, try various fallbacks.

            } else {

                // If we're in a PIE game, then use resolveStableName(actor),
                // otherwise use the actor's Unreal name.

                if (world->IsEditorWorld() && world->IsGameWorld()) {
                    #if WITH_EDITOR // defined in an auto-generated header
                        stable_name = resolveStableName(actor);
                    #else
                        SP_ASSERT(false);
                    #endif
                } else if (!world->IsEditorWorld() && world->IsGameWorld()) { 
                    stable_name = Unreal::getUnrealName(actor);
                } else {
                    SP_ASSERT(false);
                }
            }
        }
    }

    if (include_unreal_name) {
        std::string sep;
        if (stable_name != "") {
            sep = ":";
        }
        stable_name = stable_name + sep + Unreal::getUnrealName(actor);
    }

    return stable_name;
}

std::string UnrealUtils::getStableName(const UActorComponent* component, bool include_actor_stable_name, bool include_actor_unreal_name)
{
    SP_ASSERT(component);

    std::string stable_name;

    const USceneComponent* scene_component = Cast<USceneComponent>(component); // no RTTI available, so use Cast instead of dynamic_cast
    if (scene_component) {
        TArray<USceneComponent*> parents;
        scene_component->GetParentComponents(parents);
        for (auto parent : parents) {
            stable_name = Unreal::getUnrealName(parent) + "." + stable_name;
        }
    }

    stable_name = stable_name + Unreal::getUnrealName(component);

    if (include_actor_stable_name) {
        const AActor* actor = component->GetOwner();
        SP_ASSERT(actor);
        std::string actor_stable_name = getStableName(actor, include_actor_unreal_name);
        std::string sep;
        if (actor_stable_name != "") {
            sep = ":";
        }
        stable_name = actor_stable_name + sep + stable_name;
    }

    return stable_name;
}

bool UnrealUtils::canSetStableName(const AActor* actor)
{
    SP_ASSERT(actor);

    UWorld* world = actor->GetWorld();
    SP_ASSERT(world);
    SP_ASSERT(!world->IsPreviewWorld());
    SP_ASSERT(GEngine->GetWorldContextFromWorld(world));

    // If we're in the editor but outside of a PIE session, we can always set the folder path and actor label.

    if (world->IsEditorWorld() && !world->IsGameWorld()) {
        #if WITH_EDITOR // defined in an auto-generated header
            return true;
        #else
            SP_ASSERT(false);
            return false;
        #endif

    // Otherwise, we're either in a PIE game or a standalone game. In this case, we need a stateful interface
    // to store the stable name.

    } else {
        std::vector<USpStableNameComponent*> sp_stable_name_components = getComponentsByType<USpStableNameComponent>(actor);

        if (sp_stable_name_components.size() >= 1) {
            return true;
        } else {
            std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());
            return sp_stable_name_managers.size() >= 1;
        }
    }
}

void UnrealUtils::setStableName(AActor* actor, const std::string& stable_name)
{
    SP_ASSERT(actor);

    UWorld* world = actor->GetWorld();
    SP_ASSERT(world);
    SP_ASSERT(!world->IsPreviewWorld());
    SP_ASSERT(GEngine->GetWorldContextFromWorld(world));

    // If we're in the editor but outside of a PIE session, set the folder path and actor label directly.

    if (world->IsEditorWorld() && !world->IsGameWorld()) {
        #if WITH_EDITOR // defined in an auto-generated header
            auto [folder_path, actor_label] = unresolveStableName(stable_name);
            actor->SetFolderPath(Unreal::createFName(folder_path));
            actor->SetActorLabel(Unreal::toFString(actor_label));
        #else
            SP_ASSERT(false);
        #endif

    // Otherwise, we're either in a PIE game or a standalone game. In this case, we try to go through our
    // stateful interfaces.

    } else {

        std::vector<USpStableNameComponent*> sp_stable_name_components = getComponentsByType<USpStableNameComponent>(actor);

        // Try to find a UStableNameComponent on the actor. If there are multiple USpStableNameComponents, then
        // we set the stable name on all of them. A user might temporarily add multiple USpStableNameComponents
        // to an actor while editing, and we want to be tolerant of this transient state. Unlike setting a stable
        // name, getting a stable name from multiple USpStableNameComponents is ambiguous, so getStableName(...)
        // requires there to be at most one.

        if (sp_stable_name_components.size() >= 1) {
            for (auto sp_stable_name_component : sp_stable_name_components) {
                SP_ASSERT(sp_stable_name_component);
                sp_stable_name_component->setStableName(stable_name);
            }

        // Otherwise, try various fallbacks.

        } else {

            std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());

            // Try to find an ASpStableNameManager. If there are multiple ASpStableNameManagers, then we set the
            // stable name on all of them. A user might temporarily add multiple ASpStableNameManagers to a level
            // while editing, and we want to be tolerant of this transient state. Unlike setting a stable name,
            // getting a stable name from multiple ASpStableNameManagers is ambiguous, so getStableName(...)
            // requires there to be at most one.

            if (sp_stable_name_managers.size() >= 1) {
                for (auto sp_stable_name_manager : sp_stable_name_managers) {
                    SP_ASSERT(sp_stable_name_manager);

                    // If the actor is registered with the stable name manager, then use setStableName(...).

                    if (sp_stable_name_manager->hasActor(actor)) {
                        sp_stable_name_manager->setStableName(actor, stable_name);

                    // Otherwise, use addActor(...).

                    } else {
                        sp_stable_name_manager->addActor(actor, stable_name);
                    }
                }
            } else {
                SP_LOG("ERROR: Called setStableName(actor=", toStringFromPtr(actor), ", stable_name=\"", stable_name, "\") but no data structure could be found to store the stable name. Call canSetStableName(...) first.");
                SP_ASSERT(false);
            }
        }
    }
}

bool UnrealUtils::trySetStableName(AActor* actor, const std::string& stable_name)
{
    if (canSetStableName(actor)) {
        setStableName(actor, stable_name);
        return true;
    } else {
        return false;
    }
}

#if WITH_EDITOR // defined in an auto-generated header
    void UnrealUtils::requestAddOrUpdateAllStableNameActors(const UWorld* world)
    {
        SP_ASSERT(world);
        std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(world);
        for (auto sp_stable_name_manager : sp_stable_name_managers) {
            SP_ASSERT(sp_stable_name_manager);
            sp_stable_name_manager->requestAddOrUpdateAllActors();
        }
    }

    void UnrealUtils::requestAddOrUpdateStableNameActor(AActor* actor)
    {
        SP_ASSERT(actor);
        std::vector<ASpStableNameManager*> sp_stable_name_managers = findActorsByType<ASpStableNameManager>(actor->GetWorld());
        for (auto sp_stable_name_manager : sp_stable_name_managers) {
            SP_ASSERT(sp_stable_name_manager);
            sp_stable_name_manager->requestAddOrUpdateActor(actor);
        }
    }

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

    std::string UnrealUtils::resolveStableName(const AActor* actor)
    {
        SP_ASSERT(actor);

        UWorld* world = actor->GetWorld();
        SP_ASSERT(world);
        SP_ASSERT(world->IsEditorWorld());
        SP_ASSERT(!world->IsPreviewWorld());
        SP_ASSERT(GEngine->GetWorldContextFromWorld(world));

        FName folder_name = actor->GetFolderPath();
        std::string folder_string;
        if (!folder_name.IsNone()) {
            folder_string = Unreal::toStdString(folder_name);
        }
        std::string label_string = Unreal::toStdString(actor->GetActorLabel());

        if (folder_string != "") {
            return folder_string + "/" + label_string;
        } else {
            return label_string;
        }
    }

    std::pair<std::string, std::string> UnrealUtils::unresolveStableName(const std::string& stable_name)
    {
        std::vector<std::string> tokens = Std::tokenize(stable_name, "/");
        if (tokens.size() == 1) {
            return {"", tokens.at(0)};
        } else {
            std::string label = tokens.back();
            std::string folder = stable_name.substr(0, stable_name.size() - label.size() - 1);
            return {folder, label};
        }
    }
#endif

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
        std::string element_string;
        if (quote_string.empty()) {
            element_string = strings.at(i);
        } else {
            element_string = getJsonString(strings.at(i));
        }
        formatted_string += quote_string + element_string + quote_string + join_prefix_string + join_string;
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

    std::string join_string;
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
        std::string key_string = getJsonString(key_strings.at(i));
        std::string join_prefix_string;
        if (i < num_elements - 1) {
            join_prefix_string = ",";
        } else {
            join_prefix_string = "";
        }
        std::string value_string;
        if (quote_string.empty()) {
            value_string = value_strings.at(i);
        } else {
            value_string = getJsonString(value_strings.at(i));
        }
        formatted_string +=
            "\"" + key_string + "\":" + value_prefix_string + quote_string + value_string + quote_string + join_prefix_string + join_string;
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
    // On the other hand, if our property is a {enum, string, name, text, object, interface, soft reference,
    // field path}, then we expect an input string to contain a string with no quotes because this is the
    // formatting convention of getPropertyValueAsString(...), and in this case we need to add quotes to
    // construct a well-formed JSON string.
    //
    // If our property is an enum byte, treat it like a string. If it is a non-enum byte, treat it like
    // an int.

    std::string quote_string;

    if (property->IsA(FEnumProperty::StaticClass())       ||
        property->IsA(FStrProperty::StaticClass())        ||
        property->IsA(FNameProperty::StaticClass())       ||
        property->IsA(FTextProperty::StaticClass())       ||
        property->IsA(FObjectProperty::StaticClass())     ||
        property->IsA(FWeakObjectProperty::StaticClass()) ||
        property->IsA(FLazyObjectProperty::StaticClass()) ||
        property->IsA(FInterfaceProperty::StaticClass())  ||
        property->IsA(FSoftObjectProperty::StaticClass()) ||
        property->IsA(FFieldPathProperty::StaticClass())) {
        quote_string = "\"";
    } else if (property->IsA(FByteProperty::StaticClass())) {
        const FByteProperty* byte_property = static_cast<const FByteProperty*>(property);
        if (byte_property->Enum) {
            quote_string = "\"";
        }
    }

    return quote_string;
}

std::vector<std::pair<std::string, std::string>> UnrealUtils::parsePropertyPath(const std::string& property_path)
{
    // We parse a property path of the form Name0[Index0].Name1[Index1]... into a sequence of (name, index)
    // pairs, where each [Index] is optional and the index string is empty if there is no index. Each name can
    // contain any character other than '.', '[', ']', or '"'. Each index is either a JSON string literal
    // enclosed in "" quotes (e.g., "hello.world" or "a\"b"), which is returned with its enclosing quotes and
    // escape sequences intact, or an unquoted string that can contain any character other than '[', ']', or
    // '"'. We scan the path one character at a time rather than tokenizing on "." and "[]" so that quoted map
    // keys can contain these characters.

    const std::string name_delimiters = ".[]\"";
    const std::string index_delimiters = "[]\"";

    std::vector<std::pair<std::string, std::string>> path_elements;
    size_t pos = 0;

    while (true) {
        size_t name_begin = pos;
        while (pos < property_path.size() && name_delimiters.find(property_path.at(pos)) == std::string::npos) {
            pos++;
        }
        std::string name = property_path.substr(name_begin, pos - name_begin);
        SP_ASSERT(!name.empty());

        std::string index;
        if (pos < property_path.size() && property_path.at(pos) == '[') {
            pos++;
            size_t index_begin = pos;
            if (pos < property_path.size() && property_path.at(pos) == '"') {
                pos++;
                while (pos < property_path.size() && property_path.at(pos) != '"') {
                    if (property_path.at(pos) == '\\') {
                        pos++; // skip the escaped character so an escaped quote doesn't end the string
                    }
                    pos++;
                }
                SP_ASSERT(pos < property_path.size()); // expect a closing quote
                pos++;
            } else {
                while (pos < property_path.size() && index_delimiters.find(property_path.at(pos)) == std::string::npos) {
                    pos++;
                }
            }
            index = property_path.substr(index_begin, pos - index_begin);
            SP_ASSERT(!index.empty());
            SP_ASSERT(pos < property_path.size() && property_path.at(pos) == ']');
            pos++;
        }

        path_elements.push_back(std::make_pair(name, index));

        if (pos == property_path.size()) {
            break;
        }
        SP_ASSERT(property_path.at(pos) == '.');
        pos++;
    }

    return path_elements;
}

std::string UnrealUtils::getJsonString(const std::string& input)
{
    std::string output;
    output.reserve(input.size());
    for (char c : input) {
        switch (c) {
            case '"':  output += "\\\""; break;
            case '\\': output += "\\\\"; break;
            case '\b': output += "\\b";  break;
            case '\f': output += "\\f";  break;
            case '\n': output += "\\n";  break;
            case '\r': output += "\\r";  break;
            case '\t': output += "\\t";  break;
            default:   output += c;      break;
        }
    }
    return output;
}

//
// Helper functions for fixed-size array properties
//

bool UnrealUtils::isFixedSizeArrayProperty(const FProperty* property)
{
    SP_ASSERT(property);
    return property->ArrayDim > 1;
}

void UnrealUtils::validatePropertyDesc(const SpPropertyDesc& property_desc)
{
    // An SpPropertyDesc can be stored by the caller and used later, after the objects it refers to have been
    // destroyed. value_ptr_ points into memory owned by notify_objects_.back(), so we validate every entry in
    // notify_objects_ before dereferencing value_ptr_ or property_. This check can't detect a desc that refers
    // to an element of a container that has been resized since the desc was resolved.

    for (auto& notify_object : property_desc.notify_objects_) {
        toPtr<UObject>(notify_object);
    }
}

bool UnrealUtils::isFixedSizeArray(const SpPropertyDesc& property_desc, bool allow_element_to_recurse)
{
    SP_ASSERT(property_desc.property_);
    return isFixedSizeArrayProperty(property_desc.property_) && !isFixedSizeArrayElement(property_desc, allow_element_to_recurse);
}

bool UnrealUtils::isFixedSizeArrayElement(const SpPropertyDesc& property_desc, bool allow_element_to_recurse)
{
    SP_ASSERT(property_desc.property_);

    // A fixed-size array element has the same FProperty as the entire fixed-size array, so we can't tell them
    // apart by looking at the FProperty alone. If allow_element_to_recurse is false, then we're being called
    // recursively on an element of a fixed-size array. Otherwise, resolveProperty(...) records a non-negative
    // array index for the final property in the path if the user indexed into a fixed-size array. Note that the
    // Inner property of an FArrayProperty always has ArrayDim == 1, so this function never returns true for an
    // FArrayProperty element.

    if (!isFixedSizeArrayProperty(property_desc.property_)) {
        return false;
    }

    if (!allow_element_to_recurse) {
        return true;
    }

    return !property_desc.notify_array_indices_.empty() && property_desc.notify_array_indices_.back() != -1;
}

//
// Helper functions for returning dictionaries
//

std::string UnrealUtils::toKey(const UStruct* ustruct, bool include_path)
{
    SP_ASSERT(ustruct);
    std::string key = Unreal::getTypeAsString(ustruct);
    if (include_path) {
        key = key + ":" + Unreal::toStdString(ustruct->GetPathName());
    }
    return key;
}

std::string UnrealUtils::toKey(const UActorComponent* component, bool include_actor_stable_name, bool include_actor_unreal_name)
{
    return getStableName(component, include_actor_stable_name, include_actor_unreal_name);
}

std::string UnrealUtils::toKey(const AActor* actor, bool include_unreal_name)
{
    return getStableName(actor, include_unreal_name);
}

std::string UnrealUtils::toKey(const UFunction* ufunction)
{
    SP_ASSERT(ufunction);
    UStruct* outer = Cast<UStruct>(ufunction->GetOuter());
    SP_ASSERT(outer);
    return Unreal::getTypeAsString(outer) + "::" + Unreal::getUnrealName(ufunction);
}
