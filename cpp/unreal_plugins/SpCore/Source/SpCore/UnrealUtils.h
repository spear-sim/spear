//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // int32_t, uint64_t

#include <concepts> // std::derived_from, std::same_as
#include <map>
#include <ranges>   // std::views::drop, std::views::filter, std::views::transform
#include <string>   // std::stoi
#include <utility>  // std::make_pair, std::pair
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>
#include <Containers/UnrealString.h> // FString::operator*
#include <CoreGlobals.h>             // IsInGameThread
#include <Dom/JsonValue.h>
#include <EngineUtils.h>             // TActorIterator
#include <GameFramework/Actor.h>
#include <HAL/IConsoleManager.h>     // ECVF_Unregistered, IConsoleVariable
#include <HAL/Platform.h>            // SPCORE_API
#include <Templates/Casts.h>
#include <Templates/SharedPointer.h> // TSharedPtr
#include <UObject/Class.h>           // EIncludeSuperFlag
#include <UObject/Object.h>          // UObject
#include <UObject/ObjectMacros.h>    // EObjectFlags, EPropertyFlags
#include <UObject/Script.h>          // EFunctionFlags
#include <UObject/UnrealType.h>      // EFieldIterationFlags, FProperty, TFieldIterator
#include <UObject/UObjectArray.h>    // FUObjectItem, GUObjectArray
#include <UObject/UObjectGlobals.h>  // NewObject
#include <UObject/UObjectIterator.h>

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/SpTypes.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

class UClass;
class UFunction;
class UStruct;
class UScriptStruct;
class UWorld;

//
// General-purpose utility functions for working with Unreal objects. The functions here are higher-level
// than the ones in the Unreal class.
//

//
// Unreal FNames compare case-insensitively, so we design our functions that search for FNames (e.g., that match a given
// std::string) to search case-insensitively. More specifically, our functions that search for types (classes and
// structs), functions, properties, components, and tags do so case-insensitively unless otherwise noted.
//
// In SPEAR, we introduce the concept of a "stable name" for actors and components. A stable name is a human-readable
// name that is consistent across the editor, PIE sessions, and standalone games. An actor's stable name is its folder
// path and label, as displayed in the editor's World Outliner (e.g., "Meshes/05_chair/LivingRoom_Chair_01"). Actor
// stable names are based on Unreal FStrings, which compare case-sensitively. We design our functions that search for
// actors to do so according to their stable names, and therefore our functions that search for actors do so
// case-sensitively. On the other hand, component stable names are based on Unreal FNames, i.e., a component's stable
// name is the "."-separated path of FNames from its root component to itself (e.g., "DefaultSceneRoot.StaticMeshComponent0"),
// and therefore our functions that search for components do so case-insensitively. Our findActorAndComponentByName and
// findActorAndComponentByPath functions search for actor+component pairs based on input strings of the form
// "actor:component" (e.g., "Meshes/05_chair/LivingRoom_Chair_01:DefaultSceneRoot.StaticMeshComponent0"), which are
// split on the last ":", such that the actor part is compared case-sensitively, and the component part is compared
// case-insensitively.
//
// In some cases, the keys that are used to assemble the std::map objects returned by our *AsMap functions are not
// guaranteed to be unique, e.g., when calling findActorsAsMap there can be two actors with the stable name "MyActor" in
// a scene. To address this issue, our *AsMap functions drop duplicate keys according to the case-sensitivity policies
// described above. For types, keys are compared and dropped case-insensitively. For actors, keys are compared and dropped
// case-sensitively. For components, keys are compared and dropped case-insensitively if include_actor_stable_name=false.
// If include_actor_stable_name=true, then keys have the form "actor:component", and the actor part is compared
// case-sensitively, and the component part is compared case-insensitively.
//
// To guarantee that no keys are dropped when calling *AsMap functions, a user can set include_path=true for types,
// include_unreal_id=true for actors, and include_actor_stable_name=true and include_actor_unreal_id=true for components.
// Doing so will append or prepend a string to each key that guarantees uniqueness for the purpose of constructing a
// dictionary, and will therefore make it so our *AsMap functions don't drop any keys, but will also make it so the keys
// cannot be passed directly back into our *ByName functions.
//

class SPCORE_API UnrealUtils
{
public:
    UnrealUtils() = delete;
    ~UnrealUtils() = delete;

    //
    // Initialize and terminate. These functions must be called after engine initialization (see
    // SpCore::postEngineInitHandler()) and before engine exit (see SpCore::enginePreExitHandler()).
    //

    static void initialize();
    static void terminate();

    //
    // Find static structs
    //

    static std::vector<UScriptStruct*> findStaticStructs();
    static std::map<std::string, UScriptStruct*> findStaticStructsAsMap(bool include_path = false);

    //
    // Get static classes
    //

    static std::vector<UClass*> findStaticClasses();
    static std::map<std::string, UClass*> findStaticClassesAsMap(bool include_path = false);

    static std::vector<UClass*> getDerivedClasses(UClass* uclass, bool recursive = true);
    static std::map<std::string, UClass*> getDerivedClassesAsMap(UClass* uclass, bool recursive = true, bool include_path = false);

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
    // Call function
    //

    static std::map<std::string, SpPropertyValue> callFunction(
        const UWorld* world,
        UObject* uobject,
        UFunction* ufunction,
        const std::map<std::string, std::string>& args = {},
        const std::string& world_context_object = "WorldContextObject");

    //
    // Find properties
    //

    static std::vector<FProperty*> findProperties(const UStruct* ustruct, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::map<std::string, FProperty*> findPropertiesAsMap(const UStruct* ustruct, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);

    static std::vector<FProperty*> findPropertiesByName(const UStruct* ustruct, const std::string& property_name, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::vector<FProperty*> findPropertiesByFlagsAny(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::vector<FProperty*> findPropertiesByFlagsAll(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);

    static std::map<std::string, FProperty*> findPropertiesByNameAsMap(const UStruct* ustruct, const std::string& property_name, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::map<std::string, FProperty*> findPropertiesByFlagsAnyAsMap(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static std::map<std::string, FProperty*> findPropertiesByFlagsAllAsMap(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);

    static FProperty* findPropertyByName(const UStruct* ustruct, const std::string& property_name, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static FProperty* findPropertyByFlagsAny(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);
    static FProperty* findPropertyByFlagsAll(const UStruct* ustruct, EPropertyFlags property_flags, EFieldIterationFlags field_iteration_flags = EFieldIterationFlags::Default);

    //
    // Get and set multiple properties
    //

    //
    // If force_no_skip is true, then skip_property_flags is ignored and no properties are skipped.
    //

    static std::string getObjectPropertiesAsString(const UObject* uobject, EPropertyFlags check_property_flags = CPF_None, EPropertyFlags skip_property_flags = CPF_Deprecated | CPF_Transient, bool force_no_skip = false);
    static std::string getObjectPropertiesAsString(const void* value_ptr, const UStruct* ustruct, EPropertyFlags check_property_flags = CPF_None, EPropertyFlags skip_property_flags = CPF_Deprecated | CPF_Transient, bool force_no_skip = false);

    static void setObjectPropertiesFromString(UObject* uobject, const std::string& string, bool notify_editor = false, EPropertyFlags check_property_flags = CPF_None, EPropertyFlags skip_property_flags = CPF_Deprecated | CPF_Transient, bool force_no_skip = false);
    static void setObjectPropertiesFromString(void* value_ptr, const UStruct* ustruct, const std::string& string, UObject* notify_object = nullptr, EPropertyFlags check_property_flags = CPF_None, EPropertyFlags skip_property_flags = CPF_Deprecated | CPF_Transient, bool force_no_skip = false);

    //
    // Get and set individual properties. Note that uobject can't be const because we cast it to (non-const)
    // void*, value_ptr can't be const because we assign to SpPropertyDesc::value_ptr_.
    //

    static SpPropertyDesc resolveProperty(UObject* uobject, FProperty* property);
    static SpPropertyDesc resolveProperty(UObject* uobject, const std::string& property_name);
    static SpPropertyDesc resolveProperty(void* value_ptr, const UStruct* ustruct, FProperty* property, UObject* notify_object = nullptr, const SpPropertyDesc& parent = {});
    static SpPropertyDesc resolveProperty(void* value_ptr, const UStruct* ustruct, const std::string& property_name, UObject* notify_object = nullptr, const SpPropertyDesc& parent = {});
    static SpPropertyDesc resolveProperty(const SpPropertyDesc& parent, FProperty* property);
    static SpPropertyDesc resolveProperty(const SpPropertyDesc& parent, const std::string& property_name);

    // If property_desc refers to an entire fixed-size array (e.g., float Foo[4]), then getPropertyValueAsString(...)
    // and setPropertyValueFromJsonValue(...) recurse into each element. Recursive calls pass
    // allow_element_to_recurse=false to indicate that property_desc refers to a single element.

    static SpPropertyValue getPropertyValueAsString(const SpPropertyDesc& property_desc, bool allow_element_to_recurse = true);
    static void setPropertyValueFromString(const SpPropertyDesc& property_desc, const std::string& string, bool notify_editor = false);
    static void setPropertyValueFromJsonValue(const SpPropertyDesc& property_desc, TSharedPtr<FJsonValue> json_value, bool notify_editor = false, bool allow_element_to_recurse = true);

    //
    // Helper functions to fire editor property notifications. supportsEditChangeNotify returns true only if
    // a resolved SpPropertyDesc object stays within one object, has at most one array entry, and has no map
    // entries.
    //

    static bool supportsEditChangeNotify(const SpPropertyDesc& property_desc);
    static void preEditChange(const SpPropertyDesc& property_desc);
    static void postEditChange(const SpPropertyDesc& property_desc);

private:

    // setEditPropertyChain uses the out-param style because FEditPropertyChain is neither copyable nor movable, so
    // it can't be returned by value. getPropertyChangedChainEvent can be returned by value, but it holds non-owning
    // references into caller-owned backing storage that must outlive the returned event. These functions assume but
    // do not assert that supportsEditChangeNotify(property_desc) is true.

    static void setEditPropertyChain(const SpPropertyDesc& property_desc, FEditPropertyChain& property_chain);
    static FPropertyChangedChainEvent getPropertyChangedChainEvent(const SpPropertyDesc& property_desc, FEditPropertyChain& property_chain, TArray<TMap<FString, int32>>& array_indices_per_object);

public:

    //
    // Find objects
    //

    static std::vector<UObject*> findObjects();

    //
    // Find actors unconditionally and return an std::vector or an std::map
    //

    static std::vector<AActor*> findActors(const UWorld* world);
    static std::map<std::string, AActor*> findActorsAsMap(const UWorld* world, bool include_unreal_id = false);

    //
    // Get components unconditionally and return an std::vector or an std::map
    //

    static std::vector<UActorComponent*> getComponents(const AActor* actor);
    static std::map<std::string, UActorComponent*> getComponentsAsMap(const AActor* actor, bool include_actor_stable_name = false, bool include_actor_unreal_id = false);

    //
    // Get children components unconditionally and return an std::vector or an std::map (could be templated but we choose to explicitly instantiate for readability)
    //

    static std::vector<USceneComponent*> getChildrenComponents(const AActor* parent, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponents(const USceneComponent* parent, bool include_all_descendants = true);

    static std::map<std::string, USceneComponent*> getChildrenComponentsAsMap(const AActor* parent, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false);
    static std::map<std::string, USceneComponent*> getChildrenComponentsAsMap(const USceneComponent* parent, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false);

    //
    // Get and set actor and component stable names
    //

    static std::string getStableName(const AActor* actor, bool include_unreal_id = false);
    static std::string getStableName(const UActorComponent* component, bool include_actor_stable_name = false, bool include_actor_unreal_id = false);
    static bool canSetStableName(const AActor* actor);
    static void setStableName(AActor* actor, const std::string& stable_name);
    static bool trySetStableName(AActor* actor, const std::string& stable_name);

    #if WITH_EDITOR // defined in an auto-generated header
        static void requestAddOrUpdateAllStableNameActors(const UWorld* world);
        static void requestAddOrUpdateStableNameActor(AActor* actor);
        static void requestAddStableNameActor(AActor* actor);
        static void requestRemoveStableNameActor(AActor* actor);
        static void requestUpdateStableName(AActor* actor);
        static std::string resolveStableName(const AActor* actor);
        static std::pair<std::string, std::string> unresolveStableName(const std::string& stable_name);
    #endif

    //
    // Convert pointers to SpHandles. These functions must be called on the game thread, because creating an
    // SpHandle from a UObject pointer accesses GUObjectArray. A native interface (e.g., INavAgentInterface)
    // isn't a UObject, and may be located at a non-zero offset within the UObject that implements it (e.g.,
    // APawn), so an interface pointer can't be reinterpret_cast to or from a UObject pointer. So we always
    // store the address of the implementing UObject in an SpHandle, and we use Unreal's Cast<...> function to
    // convert between the UObject and the interface.
    //

    template <typename TValue>
    static SpHandle toHandle(const TValue* src)
    {
        SpHandle handle;
        handle.ptr_ = reinterpret_cast<uint64_t>(src);
        return handle;
    }

    template <CObject TObject> // specialization to store the same data as a TWeakObjectPtr for UObject pointers
    static SpHandle toHandle(const TObject* src)
    {
        SP_ASSERT(IsInGameThread());
        SpHandle handle;
        if (src) {
            handle.ptr_ = reinterpret_cast<uint64_t>(src);
            handle.object_index_ = GUObjectArray.ObjectToIndex(src);
            handle.object_serial_number_ = GUObjectArray.AllocateSerialNumber(handle.object_index_);
        }
        return handle;
    }

    template <CInterface TInterface> // specialization to store the implementing UObject for interface pointers
    static SpHandle toHandle(const TInterface* src)
    {
        return toHandle(toObjectPtr(src));
    }

    // Specialization to store the owning UObject's index and serial number for property pointers. FProperty
    // isn't a UObject, so it doesn't have an entry in GUObjectArray, but it is owned by a UStruct (or a UClass
    // or UFunction), and it lives exactly as long as its owner. So we store the property's address in ptr_, and
    // we store the owner's index and serial number in object_index_ and object_serial_number_.
    template <typename TProperty> requires std::derived_from<TProperty, FProperty>
    static SpHandle toHandle(const TProperty* src)
    {
        SpHandle handle;
        if (src) {
            SpHandle owner_handle = toHandle(src->GetOwnerUObject());
            SP_ASSERT(owner_handle.ptr_);
            handle.ptr_ = reinterpret_cast<uint64_t>(static_cast<const FProperty*>(src));
            handle.object_index_ = owner_handle.object_index_;
            handle.object_serial_number_ = owner_handle.object_serial_number_;
        }
        return handle;
    }

    template <typename TValue>
    static std::vector<SpHandle> toHandle(const std::vector<TValue*>& src)
    {
        return Std::toVector<SpHandle>(src | std::views::transform([](auto src_ptr) { return toHandle(src_ptr); }));
    }

    template <typename TKey, typename TValue>
    static std::map<TKey, SpHandle> toHandle(const std::map<TKey, TValue*>& src)
    {
        return Std::toMap<TKey, SpHandle>(src | std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, toHandle(value)); }));
    }

    //
    // Convert SpHandles to pointers. These functions must be called on the game thread, because validating a
    // UObject pointer accesses GUObjectArray.
    //

    template <typename TPtr>
    static TPtr* toPtr(const SpHandle& src)
    {
        return reinterpret_cast<TPtr*>(src.ptr_);
    }

    template <CObject TObject> // specialization to validate UObject pointers
    static TObject* toPtr(const SpHandle& src)
    {
        SP_ASSERT(IsInGameThread());
        TObject* src_ptr = reinterpret_cast<TObject*>(src.ptr_);
        if (src_ptr) {

            // If the handle has a serial number, then validate it in the same way as TWeakObjectPtr, and check
            // that the object at object_index_ is the object that src_ptr points to. Otherwise, the handle was
            // created without a serial number (e.g., from a uint64_t or a bare hexadecimal pointer string), so we
            // fall back to checking src_ptr directly.

            if (src.object_serial_number_ != 0) {
                FUObjectItem* object_item = GUObjectArray.IndexToObject(src.object_index_);
                SP_ASSERT(object_item);
                SP_ASSERT(object_item->Object == src_ptr); // TODO: replace with object_item->GetObject() in UE 5.8
                SP_ASSERT(object_item->GetSerialNumber() == src.object_serial_number_);
                SP_ASSERT(GUObjectArray.IsValid(object_item, false));
            }
            SP_ASSERT(Unreal::isValid(src_ptr));
        }
        return src_ptr;
    }

    template <CInterface TInterface> // specialization to validate the implementing UObject for interface pointers
    static TInterface* toPtr(const SpHandle& src)
    {
        return toInterfacePtr<TInterface>(toPtr<UObject>(src));
    }

    // Specialization to validate the owning UObject for property pointers. See the corresponding toHandle(...)
    // specialization above. This check can detect a property whose owner has been destroyed, but it can't
    // detect a property that has been destroyed while its owner remains alive (e.g., if a Blueprint class is
    // recompiled in place). If the handle doesn't have a serial number (e.g., if it was created from a
    // uint64_t), then we can't validate it.
    template <typename TProperty> requires std::derived_from<TProperty, FProperty>
    static TProperty* toPtr(const SpHandle& src)
    {
        TProperty* src_ptr = reinterpret_cast<TProperty*>(src.ptr_);
        if (src_ptr && src.object_serial_number_ != 0) {
            SP_ASSERT(IsInGameThread());
            FUObjectItem* owner_item = GUObjectArray.IndexToObject(src.object_index_);
            SP_ASSERT(owner_item);
            SpHandle owner_handle;
            owner_handle.ptr_ = reinterpret_cast<uint64_t>(owner_item->Object); // TODO: replace with owner_item->GetObject() in UE 5.8
            owner_handle.object_index_ = src.object_index_;
            owner_handle.object_serial_number_ = src.object_serial_number_;
            UObject* owner = toPtr<UObject>(owner_handle);
            SP_ASSERT(owner);
            SP_ASSERT(src_ptr->GetOwnerUObject() == owner);
        }
        return src_ptr;
    }

    // Specialization to validate console variable pointers. IConsoleVariable isn't a UObject, but unregistering
    // a console variable usually doesn't delete it. Instead, Unreal keeps it alive and sets ECVF_Unregistered,
    // so we can detect a stale handle in the common case by checking this flag.
    template <typename TPtr> requires std::same_as<TPtr, IConsoleVariable>
    static TPtr* toPtr(const SpHandle& src)
    {
        IConsoleVariable* src_ptr = reinterpret_cast<IConsoleVariable*>(src.ptr_);
        if (src_ptr) {
            SP_ASSERT(!(src_ptr->GetFlags() & ECVF_Unregistered));
        }
        return src_ptr;
    }

    template <typename TPtr> // specialization not needed because we delegate to the scalar implementation
    static std::vector<TPtr*> toPtr(const std::vector<SpHandle>& src)
    {
        return Std::toVector<TPtr*>(src | std::views::transform([](const auto& handle) { return toPtr<TPtr>(handle); }));
    }

    template <typename TKey, typename TPtr> // specialization not needed because we delegate to the scalar implementation
    static std::map<TKey, TPtr*> toPtr(const std::map<TKey, SpHandle>& src)
    {
        return Std::toMap<TKey, TPtr*>(src | std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, toPtr<TPtr>(value)); }));
    }

    //
    // Convert between UObject pointers and strings. These functions are used to pass UObject pointers to and
    // from UFUNCTIONs and properties via JSON. A pointer string has the form "0x<ptr>:<object_index>:<object_serial_number>",
    // so it can be validated in the same way as an SpHandle. A bare hexadecimal pointer string of the form
    // "0x<ptr>" is also accepted, but it can't be validated as strongly. These functions must be called on the
    // game thread, because they call toHandle(...) and toPtr(...).
    //

    template <CObject TObject>
    static std::string toStringFromPtr(const TObject* src)
    {
        SpHandle handle = toHandle(src);
        return Std::toStringFromPtr(src) + ":" + Std::toString(handle.object_index_) + ":" + Std::toString(handle.object_serial_number_);
    }

    template <CObject TObject>
    static TObject* toPtrFromString(const std::string& src)
    {
        std::vector<std::string> tokens = Std::tokenize(src, ":");
        SP_ASSERT(tokens.size() == 1 || tokens.size() == 3);
        SpHandle handle;
        handle.ptr_ = reinterpret_cast<uint64_t>(Std::toPtrFromString<void>(tokens.at(0)));
        if (tokens.size() == 3) {
            handle.object_index_ = std::stoi(tokens.at(1));
            handle.object_serial_number_ = std::stoi(tokens.at(2));
        }
        return toPtr<TObject>(handle);
    }

    //
    // Convert between pointers and uint64_t. These functions are useful in cases where pointers need to be
    // stored as numeric data (e.g., in an SpArray<uint64_t>). UObject pointers obtained this way are validated
    // with Unreal::isValid(...), which is weaker than the validation that is possible with an SpHandle. As with
    // SpHandles, interface pointers are stored as the address of the implementing UObject.
    //

    template <typename TValue>
    static uint64_t toUInt64(const TValue* src)
    {
        return reinterpret_cast<uint64_t>(src);
    }

    template <CInterface TInterface> // specialization to store the implementing UObject for interface pointers
    static uint64_t toUInt64(const TInterface* src)
    {
        return toUInt64(toObjectPtr(src));
    }

    template <typename TValue>
    static std::vector<uint64_t> toUInt64(std::vector<TValue>&& src)
    {
        return Std::reinterpretAsVectorOf<uint64_t>(std::move(src));
    }

    template <CInterface TInterface> // specialization to store the implementing UObject for interface pointers
    static std::vector<uint64_t> toUInt64(std::vector<TInterface*>&& src)
    {
        return Std::toVector<uint64_t>(src | std::views::transform([](auto src_ptr) { return toUInt64(src_ptr); }));
    }

    template <typename TKey, typename TValue>
    static std::map<TKey, uint64_t> toUInt64(const std::map<TKey, TValue>& src)
    {
        return Std::toMap<TKey, uint64_t>(src | std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, toUInt64(value)); }));
    }

    template <typename TPtr>
    static TPtr* toPtr(uint64_t src)
    {
        return reinterpret_cast<TPtr*>(src);
    }

    template <CObject TObject> // specialization to validate UObject pointers
    static TObject* toPtr(uint64_t src)
    {
        SP_ASSERT(IsInGameThread());
        TObject* src_ptr = reinterpret_cast<TObject*>(src);
        if (src_ptr) {
            SP_ASSERT(Unreal::isValid(src_ptr));
        }
        return src_ptr;
    }

    template <CInterface TInterface> // specialization to validate the implementing UObject for interface pointers
    static TInterface* toPtr(uint64_t src)
    {
        return toInterfacePtr<TInterface>(toPtr<UObject>(src));
    }

    template <typename TPtr>
    static std::vector<TPtr*> toPtr(std::vector<uint64_t>&& src)
    {
        return Std::reinterpretAsVectorOf<TPtr*>(std::move(src));
    }

    template <CObject TObject> // specialization to validate UObject pointers
    static std::vector<TObject*> toPtr(std::vector<uint64_t>&& src)
    {
        SP_ASSERT(IsInGameThread());
        std::vector<TObject*> dest = Std::reinterpretAsVectorOf<TObject*>(std::move(src));
        for (auto dest_ptr : dest) {
            if (dest_ptr) {
                SP_ASSERT(Unreal::isValid(dest_ptr));
            }
        }
        return dest;
    }

    template <CInterface TInterface> // specialization to validate the implementing UObject for interface pointers
    static std::vector<TInterface*> toPtr(std::vector<uint64_t>&& src)
    {
        return Std::toVector<TInterface*>(src | std::views::transform([](auto src_uint64) { return toPtr<TInterface>(src_uint64); }));
    }

    template <typename TKey, typename TPtr> // specialization not needed because we delegate to the scalar implementation
    static std::map<TKey, TPtr*> toPtr(const std::map<TKey, uint64_t>& src)
    {
        return Std::toMap<TKey, TPtr*>(src | std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, toPtr<TPtr>(value)); }));
    }

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
            std::views::filter([&static_struct_name](auto static_struct) { return Std::toLower(Unreal::getTypeAsString(static_struct)) == Std::toLower(static_struct_name); })); // case-insensitive comparison
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
    static std::map<std::string, TReturnAsStaticStruct*> findStaticStructsByNameAsMap(const std::string& static_struct_name, bool include_path = false)
    {
        return toMap(findStaticStructsByName<TStaticStruct, TReturnAsStaticStruct>(static_struct_name), include_path);
    }

    template <CStaticStruct TStaticStruct, CStaticStruct TReturnAsStaticStruct = TStaticStruct> requires
        std::derived_from<TStaticStruct, TReturnAsStaticStruct>
    static std::map<std::string, TReturnAsStaticStruct*> findStaticStructsByTypeAsMap(bool include_path = false)
    {
        return toMap(findStaticStructsByType<TStaticStruct, TReturnAsStaticStruct>(), include_path);
    }

    // UClass* variants

    template <CStaticStruct TReturnAsStaticStruct = UStruct>
    static std::map<std::string, TReturnAsStaticStruct*> findStaticStructsByNameAsMap(UClass* uclass, const std::string& static_struct_name, bool include_path = false)
    {
        return toMap(findStaticStructsByName<TReturnAsStaticStruct>(uclass, static_struct_name), include_path);
    }

    template <CStaticStruct TReturnAsStaticStruct = UStruct>
    static std::map<std::string, TReturnAsStaticStruct*> findStaticStructsByClassAsMap(UClass* uclass, bool include_path = false)
    {
        return toMap(findStaticStructsByClass<TReturnAsStaticStruct>(uclass), include_path);
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
            std::views::filter([&tags](auto actor) { return Std::any(Std::contains(toLower(Unreal::getTags(actor)), toLower(tags))); })); // case-insensitive comparison
    }

    template <CActor TReturnAsActor = AActor>
    static std::vector<TReturnAsActor*> findActorsByTagsAll(UClass* uclass, const UWorld* world, const std::vector<std::string>& tags)
    {
        return Std::toVector<TReturnAsActor*>(
            findActorsByClass<TReturnAsActor>(uclass, world) |
            std::views::filter([&tags](auto actor) { return Std::all(Std::contains(toLower(Unreal::getTags(actor)), toLower(tags))); })); // case-insensitive comparison
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
    static std::map<std::string, TReturnAsActor*> findActorsByNameAsMap(const UWorld* world, const std::string& actor_name, bool include_unreal_id = false)
    {
        return toMap(findActorsByName<TActor, TReturnAsActor>(world, actor_name), include_unreal_id);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAsMap(const UWorld* world, const std::string& tag, bool include_unreal_id = false)
    {
        return toMap(findActorsByTag<TActor, TReturnAsActor>(world, tag), include_unreal_id);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagsAnyAsMap(const UWorld* world, const std::vector<std::string>& tags, bool include_unreal_id = false)
    {
        return toMap(findActorsByTagsAny<TActor, TReturnAsActor>(world, tags), include_unreal_id);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagsAllAsMap(const UWorld* world, const std::vector<std::string>& tags, bool include_unreal_id = false)
    {
        return toMap(findActorsByTagsAll<TActor, TReturnAsActor>(world, tags), include_unreal_id);
    }

    template <CActor TActor = AActor, CActor TReturnAsActor = TActor> requires
        std::derived_from<TActor, TReturnAsActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTypeAsMap(const UWorld* world, bool include_unreal_id = false)
    {
        return toMap(findActorsByType<TActor, TReturnAsActor>(world), include_unreal_id);
    }

    // UClass* variants

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByNameAsMap(UClass* uclass, const UWorld* world, const std::string& actor_name, bool include_unreal_id = false)
    {
        return toMap(findActorsByName<TReturnAsActor>(uclass, world, actor_name), include_unreal_id);
    }

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagAsMap(UClass* uclass, const UWorld* world, const std::string& tag, bool include_unreal_id = false)
    {
        return toMap(findActorsByTag<TReturnAsActor>(uclass, world, tag), include_unreal_id);
    }

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagsAnyAsMap(UClass* uclass, const UWorld* world, const std::vector<std::string>& tags, bool include_unreal_id = false)
    {
        return toMap(findActorsByTagsAny<TReturnAsActor>(uclass, world, tags), include_unreal_id);
    }

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByTagsAllAsMap(UClass* uclass, const UWorld* world, const std::vector<std::string>& tags, bool include_unreal_id = false)
    {
        return toMap(findActorsByTagsAll<TReturnAsActor>(uclass, world, tags), include_unreal_id);
    }

    template <CActor TReturnAsActor = AActor>
    static std::map<std::string, TReturnAsActor*> findActorsByClassAsMap(UClass* uclass, const UWorld* world, bool include_unreal_id = false)
    {
        return toMap(findActorsByClass<TReturnAsActor>(uclass, world), include_unreal_id);
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
            std::views::filter([&component_name](auto component) { return Std::toLower(getStableName(component)) == Std::toLower(component_name); })); // case-insensitive comparison
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
            std::views::filter([&tags](auto component) { return Std::any(Std::contains(toLower(Unreal::getTags(component)), toLower(tags))); })); // case-insensitive comparison
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::vector<TReturnAsComponent*> getComponentsByTagsAll(UClass* uclass, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false)
    {
        return Std::toVector<TReturnAsComponent*>(
            getComponentsByClass<TReturnAsComponent>(uclass, actor, include_from_child_actors) |
            std::views::filter([&tags](auto component) { return Std::all(Std::contains(toLower(Unreal::getTags(component)), toLower(tags))); })); // case-insensitive comparison
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
    static std::map<std::string, TReturnAsComponent*> getComponentsByNameAsMap(const AActor* actor, const std::string& component_name, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByName<TComponent, TReturnAsComponent>(actor, component_name, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByPathAsMap(const AActor* actor, const std::string& component_path, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByPath<TComponent, TReturnAsComponent>(actor, component_path, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAsMap(const AActor* actor, const std::string& tag, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByTag<TComponent, TReturnAsComponent>(actor, tag, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagsAnyAsMap(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByTagsAny<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagsAllAsMap(const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByTagsAll<TComponent, TReturnAsComponent>(actor, tags, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TComponent = UActorComponent, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TComponent, TReturnAsComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTypeAsMap(const AActor* actor, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByType<TComponent, TReturnAsComponent>(actor, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    // UClass* variants

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByNameAsMap(UClass* uclass, const AActor* actor, const std::string& component_name, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByName<TReturnAsComponent>(uclass, actor, component_name, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByPathAsMap(UClass* uclass, const AActor* actor, const std::string& component_path, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByPath<TReturnAsComponent>(uclass, actor, component_path, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagAsMap(UClass* uclass, const AActor* actor, const std::string& tag, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByTag<TReturnAsComponent>(uclass, actor, tag, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagsAnyAsMap(UClass* uclass, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByTagsAny<TReturnAsComponent>(uclass, actor, tags, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByTagsAllAsMap(UClass* uclass, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByTagsAll<TReturnAsComponent>(uclass, actor, tags, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CComponent TReturnAsComponent = UActorComponent>
    static std::map<std::string, TReturnAsComponent*> getComponentsByClassAsMap(UClass* uclass, const AActor* actor, bool include_from_child_actors = false, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getComponentsByClass<TReturnAsComponent>(uclass, actor, include_from_child_actors), include_actor_stable_name, include_actor_unreal_id);
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
            std::views::filter([&child_component_name](auto component) { return Std::toLower(getStableName(component)) == Std::toLower(child_component_name); })); // case-insensitive comparison
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
            std::views::filter([&tags](auto component) { return Std::any(Std::contains(toLower(Unreal::getTags(component)), toLower(tags))); })); // case-insensitive comparison
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::vector<TReturnAsSceneComponent*> getChildrenComponentsByTagsAll(UClass* uclass, const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true)
    {
        return Std::toVector<TReturnAsSceneComponent*>(
            getChildrenComponentsByClass<TReturnAsSceneComponent>(uclass, parent, include_all_descendants) |
            std::views::filter([&tags](auto component) { return Std::all(Std::contains(toLower(Unreal::getTags(component)), toLower(tags))); })); // case-insensitive comparison
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
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByNameAsMap(const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByName<TSceneComponent, TReturnAsSceneComponent>(parent, child_component_name, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagAsMap(const TParent* parent, const std::string& tag, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByTag<TSceneComponent, TReturnAsSceneComponent>(parent, tag, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagsAnyAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByTagsAny<TSceneComponent, TReturnAsSceneComponent>(parent, tags, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagsAllAsMap(const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByTagsAll<TSceneComponent, TReturnAsSceneComponent>(parent, tags, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CSceneComponent TSceneComponent = USceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent, CParent TParent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTypeAsMap(const TParent* parent, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByType<TSceneComponent, TReturnAsSceneComponent>(parent, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
    }

    // UClass* variants

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByNameAsMap(UClass* uclass, const TParent* parent, const std::string& child_component_name, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByName<TReturnAsSceneComponent>(uclass, parent, child_component_name, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagAsMap(UClass* uclass, const TParent* parent, const std::string& tag, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByTag<TReturnAsSceneComponent>(uclass, parent, tag, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagsAnyAsMap(UClass* uclass, const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByTagsAny<TReturnAsSceneComponent>(uclass, parent, tags, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByTagsAllAsMap(UClass* uclass, const TParent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByTagsAll<TReturnAsSceneComponent>(uclass, parent, tags, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
    }

    template <CSceneComponent TReturnAsSceneComponent = USceneComponent, CParent TParent>
    static std::map<std::string, TReturnAsSceneComponent*> getChildrenComponentsByClassAsMap(UClass* uclass, const TParent* parent, bool include_all_descendants = true, bool include_actor_stable_name = false, bool include_actor_unreal_id = false)
    {
        return toMap(getChildrenComponentsByClass<TReturnAsSceneComponent>(uclass, parent, include_all_descendants), include_actor_stable_name, include_actor_unreal_id);
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
        UActorComponent* component = Cast<UActorComponent>(owner->CreateDefaultSubobject(Unreal::createFName(component_name), component_class, component_class, true, false)); // no RTTI available
        SP_ASSERT(component);
        return component;
    }

    static USceneComponent* createSceneComponentInsideOwnerConstructorByClass(UClass* scene_component_class, AActor* owner, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        USceneComponent* scene_component = Cast<USceneComponent>(owner->CreateDefaultSubobject(Unreal::createFName(scene_component_name), scene_component_class, scene_component_class, true, false)); // no RTTI available
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        return scene_component;
    }

    static USceneComponent* createSceneComponentInsideOwnerConstructorByClass(UClass* scene_component_class, UObject* owner, USceneComponent* parent, const std::string& scene_component_name)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        USceneComponent* scene_component = Cast<USceneComponent>(owner->CreateDefaultSubobject(Unreal::createFName(scene_component_name), scene_component_class, scene_component_class, true, false)); // no RTTI available
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
    static TReturnAsNonSceneComponent* createComponentOutsideOwnerConstructor(AActor* owner, const std::string& component_name, EObjectFlags object_flags = RF_NoFlags)
    {
        return Cast<TReturnAsNonSceneComponent>(createComponentOutsideOwnerConstructorByClass(TNonSceneComponent::StaticClass(), owner, component_name, object_flags)); // no RTTI available
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createSceneComponentOutsideOwnerConstructor(AActor* owner, const std::string& scene_component_name, EObjectFlags object_flags = RF_NoFlags)
    {
        return Cast<TReturnAsSceneComponent>(createSceneComponentOutsideOwnerConstructorByClass(TSceneComponent::StaticClass(), owner, scene_component_name, object_flags)); // no RTTI available
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createSceneComponentOutsideOwnerConstructor(UObject* owner, USceneComponent* parent, const std::string& scene_component_name, EObjectFlags object_flags = RF_NoFlags)
    {
        return Cast<TReturnAsSceneComponent>(createSceneComponentOutsideOwnerConstructorByClass(TSceneComponent::StaticClass(), owner, parent, scene_component_name, object_flags)); // no RTTI available
    }

    template <CSceneComponent TSceneComponent, CSceneComponent TReturnAsSceneComponent = TSceneComponent> requires
        std::derived_from<TSceneComponent, TReturnAsSceneComponent>
    static TReturnAsSceneComponent* createSceneComponentOutsideOwnerConstructor(USceneComponent* owner, const std::string& scene_component_name, EObjectFlags object_flags = RF_NoFlags)
    {
        return createSceneComponentOutsideOwnerConstructor<TSceneComponent, TReturnAsSceneComponent>(owner, owner, scene_component_name, object_flags);
    }

    static UActorComponent* createComponentOutsideOwnerConstructorByClass(UClass* component_class, AActor* owner, const std::string& component_name, EObjectFlags object_flags = RF_NoFlags)
    {
        SP_ASSERT(owner);
        UActorComponent* component = NewObject<UActorComponent>(owner, component_class, Unreal::createFName(component_name), object_flags);
        SP_ASSERT(component);
        component->RegisterComponent();
        return component;
    }

    static USceneComponent* createSceneComponentOutsideOwnerConstructorByClass(UClass* scene_component_class, AActor* owner, const std::string& scene_component_name, EObjectFlags object_flags = RF_NoFlags)
    {
        SP_ASSERT(owner);
        USceneComponent* scene_component = NewObject<USceneComponent>(owner, scene_component_class, Unreal::createFName(scene_component_name), object_flags);
        SP_ASSERT(scene_component);
        owner->SetRootComponent(scene_component);
        scene_component->RegisterComponent();
        return scene_component;
    }

    static USceneComponent* createSceneComponentOutsideOwnerConstructorByClass(UClass* scene_component_class, UObject* owner, USceneComponent* parent, const std::string& scene_component_name, EObjectFlags object_flags = RF_NoFlags)
    {
        SP_ASSERT(owner);
        SP_ASSERT(parent);
        USceneComponent* scene_component = NewObject<USceneComponent>(owner, scene_component_class, Unreal::createFName(scene_component_name), object_flags);
        SP_ASSERT(scene_component);
        scene_component->SetupAttachment(parent);
        scene_component->RegisterComponent();
        return scene_component;
    }

    static USceneComponent* createSceneComponentOutsideOwnerConstructorByClass(UClass* scene_component_class, USceneComponent* owner, const std::string& scene_component_name, EObjectFlags object_flags = RF_NoFlags)
    {
        return createSceneComponentOutsideOwnerConstructorByClass(scene_component_class, owner, owner, scene_component_name, object_flags);
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
    // Helper functions for converting between interface pointers and the UObjects that implement them. In both
    // directions, we check that converting back yields the original pointer.
    //

    template <CInterface TInterface>
    static UObject* toObjectPtr(const TInterface* src)
    {
        if (src) {
            UObject* object_ptr = src->_getUObject();
            SP_ASSERT(object_ptr);
            SP_ASSERT(Cast<TInterface>(object_ptr) == src);
            return object_ptr;
        } else {
            return nullptr;
        }
    }

    template <CInterface TInterface>
    static TInterface* toInterfacePtr(UObject* src)
    {
        if (src) {
            TInterface* interface_ptr = Cast<TInterface>(src);
            SP_ASSERT(interface_ptr);
            SP_ASSERT(interface_ptr->_getUObject() == src);
            return interface_ptr;
        } else {
            return nullptr;
        }
    }

    //
    // Helper functions for formatting container properties as strings in the same style as Unreal
    //

    static std::string getArrayPropertyValueAsFormattedString(const FProperty* property, const std::vector<std::string>& strings);
    static std::string getMapPropertyValueAsFormattedString(const FProperty* key_property, const std::vector<std::string>& key_strings, const FProperty* value_property, const std::vector<std::string>& value_strings);
    static std::string getQuoteStringForProperty(const FProperty* property);
    static std::string getJsonString(const std::string& input);

    //
    // Helper function for validating an SpPropertyDesc that might have been stored by the caller
    //

    static void validatePropertyDesc(const SpPropertyDesc& property_desc);

    //
    // Helper functions for fixed-size array properties
    //

    static bool isFixedSizeArrayProperty(const FProperty* property);
    static bool isFixedSizeArray(const SpPropertyDesc& property_desc, bool allow_element_to_recurse);
    static bool isFixedSizeArrayElement(const SpPropertyDesc& property_desc, bool allow_element_to_recurse);

    //
    // Helper function for parsing property paths
    //

    static std::vector<std::pair<std::string, std::string>> parsePropertyPath(const std::string& property_path);

    //
    // Helper functions for returning dictionaries
    //

    static std::string toKey(const UStruct* ustruct, bool include_path = false);
    static std::string toKey(const UActorComponent* component, bool include_actor_stable_name = false, bool include_actor_unreal_id = false);
    static std::string toKey(const AActor* actor, bool include_unreal_id = false);
    static std::string toKey(const UFunction* ufunction);

    template <CMappableNameObject TMappableNameObject>
    static std::string toKey(const TMappableNameObject* mappable_name_object)
    {
        return Unreal::getUnrealName(mappable_name_object);
    }

    //
    // Helper functions for finding actors and getting components and getting properties
    //

    template <CActor TActor = AActor, CComponent TComponent = UActorComponent, CActor TReturnAsActor = TActor, CComponent TReturnAsComponent = TComponent> requires
        std::derived_from<TActor, TReturnAsActor> && std::derived_from<TComponent, TReturnAsComponent>
    static std::pair<TReturnAsActor*, TReturnAsComponent*> findActorAndComponent(const UWorld* world, const AActor* owner, const std::string& path, bool get_component_by_path)
    {
        std::pair<TReturnAsActor*, TReturnAsComponent*> pair = std::make_pair(nullptr, nullptr);

        // Actor stable names can contain ":", but component names can't, so we split path on the last ":".
        size_t pos = path.rfind(":");

        TReturnAsActor* actor = nullptr;
        TReturnAsComponent* component = nullptr;
        std::string component_path;

        if (pos == std::string::npos) {
            actor = const_cast<TReturnAsActor*>(Cast<TReturnAsActor>(owner)); // no RTTI available
            component_path = path;
        } else {
            actor = findActorByName<TActor, TReturnAsActor>(world, path.substr(0, pos));
            component_path = path.substr(pos + 1);
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

    template <CMappable TMappable>
    static std::map<std::string, TMappable*> toMap(const std::vector<TMappable*>& objects)
    {
        std::vector<std::string> names = Std::toVector<std::string>(
            objects |
            std::views::transform([](auto object) { return toKey(object); }));

        return Std::zip(names, objects);
    }

    // Blueprint types are named according to their asset names, which are only unique within a content folder, so two
    // loaded blueprint types can have the same name. Analogous to the CActor overload below, we drop all types whose
    // names collide unless include_path is true, in which case every name is guaranteed to be case-insensitively
    // unique. Type names are FNames, which compare case-insensitively, so names that differ only by case also collide.

    template <CStaticStruct TStaticStruct>
    static std::map<std::string, TStaticStruct*> toMap(const std::vector<TStaticStruct*>& static_structs, bool include_path)
    {
        std::vector<std::string> names = Std::toVector<std::string>(
            static_structs |
            std::views::transform([include_path](auto static_struct) { return toKey(static_struct, include_path); }));

        bool names_are_case_insensitive_unique = include_path;
        bool case_insensitive_filter = true;
        return toNameMap(names, static_structs, names_are_case_insensitive_unique, case_insensitive_filter);
    }

    template <CActor TActor>
    static std::map<std::string, TActor*> toMap(const std::vector<TActor*>& actors, bool include_unreal_id)
    {
        std::vector<std::string> names = Std::toVector<std::string>(actors | std::views::transform([include_unreal_id](auto actor) { return toKey(actor, include_unreal_id); }));

        bool names_are_case_insensitive_unique = include_unreal_id;
        bool case_insensitive_filter = false;
        return toNameMap(names, actors, names_are_case_insensitive_unique, case_insensitive_filter);
    }

    // Actor stable names aren't guaranteed to be unique, so if components come from multiple actors, then two
    // components can have the same key. Analogous to the CActor overload above, we drop all components whose keys
    // collide. If include_actor_stable_name is true, then each key has the form "actor:component", so we compare the
    // actor part case-sensitively and the component part case-insensitively, consistent with
    // findActorAndComponentBy*(...).

    template <CComponent TComponent>
    static std::map<std::string, TComponent*> toMap(const std::vector<TComponent*>& components, bool include_actor_stable_name, bool include_actor_unreal_id)
    {
        std::vector<std::string> names = Std::toVector<std::string>(
            components |
            std::views::transform([include_actor_stable_name, include_actor_unreal_id](auto component) { return toKey(component, include_actor_stable_name, include_actor_unreal_id); }));

        if (include_actor_unreal_id) {

            // If include_actor_unreal_id is true, then each key has the form "actor:unreal_id:component". Each
            // actor's Unreal ID is case-insensitively unique, and each component's stable name is case-insensitively
            // unique within its actor (because both are built from FNames). So every key is case-insensitively
            // unique, and therefore no keys can collide under any comparison, including the mixed comparison below.
            // So we can skip the collision check without changing which components are returned.
            bool names_are_case_insensitive_unique = true;
            bool case_insensitive_filter = true;
            return toNameMap(names, components, names_are_case_insensitive_unique, case_insensitive_filter);

        } else if (include_actor_stable_name) {

            // If an actor's stable name is empty (which can happen because it is allowed to set an actor's stable name
            // to ""), then its components' keys don't contain ":", so we compare the entire key case-insensitively
            // because there is nothing to compare case-sensitively.
            auto get_key = [](const std::string& name) -> std::string {
                size_t pos = name.rfind(":");
                if (pos == std::string::npos) {
                    return Std::toLower(name); // case-insensitive comparison
                } else {
                    return name.substr(0, pos + 1) + Std::toLower(name.substr(pos + 1)); // case-sensitive actor part, case-insensitive component part
                }
            };

            std::map<std::string, uint64_t> counts = Std::count(Std::toVector<std::string>(names | std::views::transform(get_key)));
            return Std::toMap<std::string, TComponent*>(
                Std::zip(components, names) |
                std::views::filter([&counts, &get_key](const auto& pair) { return counts.at(get_key(pair.second)) == 1; }) |
                std::views::transform([](const auto& pair) { return std::make_pair(pair.second, pair.first); }));

        } else {
            bool names_are_case_insensitive_unique = false;
            bool case_insensitive_filter = true;
            return toNameMap(names, components, names_are_case_insensitive_unique, case_insensitive_filter);
        }
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

    // Returns a map from names to objects. If names_are_case_insensitive_unique is false, then we drop all objects
    // whose names collide, and if case_insensitive_filter is true, then names that differ only by case also collide. If
    // names_are_case_insensitive_unique is true, then no names can collide under either comparison, so we skip the
    // collision check.
    template <typename TObject>
    static std::map<std::string, TObject*> toNameMap(const std::vector<std::string>& names, const std::vector<TObject*>& objects, bool names_are_case_insensitive_unique, bool case_insensitive_filter)
    {
        if (names_are_case_insensitive_unique) {
            return Std::zip(names, objects);
        } else {
            if (case_insensitive_filter) {
                std::map<std::string, uint64_t> counts = Std::count(toLower(names));
                return Std::toMap<std::string, TObject*>(
                    Std::zip(objects, names) |
                    std::views::filter([&counts](const auto& pair) { return counts.at(Std::toLower(pair.second)) == 1; }) | // case-insensitive comparison
                    std::views::transform([](const auto& pair) { return std::make_pair(pair.second, pair.first); }));
            } else {
                std::map<std::string, uint64_t> counts = Std::count(names);
                return Std::toMap<std::string, TObject*>(
                    Std::zip(objects, names) |
                    std::views::filter([&counts](const auto& pair) { return counts.at(pair.second) == 1; }) |
                    std::views::transform([](const auto& pair) { return std::make_pair(pair.second, pair.first); }));
            }
        }
    }

    static std::vector<std::string> toLower(const std::vector<std::string>& strings)
    {
        return Std::toVector<std::string>(strings | std::views::transform([](const auto& string) { return Std::toLower(string); }));
    }

    inline static EPropertyFlags s_unused_property_flag_ = CPF_None;
};
