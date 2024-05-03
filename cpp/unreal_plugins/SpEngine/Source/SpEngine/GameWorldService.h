//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/World.h>                // FWorldDelegates
#include <Kismet/GameplayStatics.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Rpclib.h"
#include "SpCore/Unreal.h"
#include "SpEngine/EntryPointBinder.h"

// We use MSGPACK macros here to define structs that can be passed into, and returned from, the service entry
// points defined below. There are already similar struct defined in SpCore, but we choose to define separate
// structs here for the following reasons. First, we avoid a dependency on RPCLib in SpCore. Second, the data
// needed at the SpCore level is sometimes slightly different from the data needed at the SpEngine level, e.g.,
// PropertyDesc needs to maintain a FProperty pointer but GameWorldServicePropertyDesc doesn't. Third, some
// data types used in SpCore cannot be sent via RPCLib, e.g., FProperty* in PropertyDesc.

MSGPACK_ADD_ENUM(EIncludeSuperFlag::Type);

struct GameWorldServicePropertyDesc
{
    uint64_t property_;
    uint64_t value_ptr_;

    MSGPACK_DEFINE_MAP(property_, value_ptr_);
};

class GameWorldService {
public:
    GameWorldService() = delete;
    GameWorldService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &GameWorldService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &GameWorldService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_game_paused",
            [this](const bool& paused) -> void {
                UGameplayStatics::SetGamePaused(world_, paused);
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "open_level",
            [this](const std::string& level_name) -> void {
                UGameplayStatics::OpenLevel(world_, Unreal::toFName(level_name));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_world_name",
            [this]() -> std::string {
                SP_ASSERT(world_);
                return Unreal::toStdString(world_->GetName());
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors",
            [this]() -> std::vector<uint64_t> {
                return toUint64(Unreal::findActors(world_));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_as_map",
            [this]() -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::findActorsAsMap(world_));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components",
            [this](const uint64_t& actor) -> std::vector<uint64_t> {
                return toUint64(Unreal::getComponents(reinterpret_cast<AActor*>(actor)));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_as_map",
            [this](const uint64_t& actor) -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::getComponentsAsMap(reinterpret_cast<AActor*>(actor)));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components",
            [this](const uint64_t& parent, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUint64(Unreal::getChildrenComponents(reinterpret_cast<USceneComponent*>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_as_map",
            [this](const uint64_t& parent, const bool& include_all_descendants) -> std::map<std::string, uint64_t> {
                return toUint64(Unreal::getChildrenComponentsAsMap(reinterpret_cast<USceneComponent*>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_object_properties_as_string_from_uobject",
            [this](const uint64_t& uobject) -> std::string {
                return Unreal::getObjectPropertiesAsString(reinterpret_cast<UObject*>(uobject));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_object_properties_as_string_from_ustruct",
            [this](const uint64_t& value_ptr, const uint64_t& ustruct) -> std::string {
                return Unreal::getObjectPropertiesAsString(reinterpret_cast<void*>(value_ptr), reinterpret_cast<UStruct*>(ustruct));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_object_properties_from_string_for_uobject",
            [this](const uint64_t& uobject, const std::string& string) -> void {
                Unreal::setObjectPropertiesFromString(reinterpret_cast<UObject*>(uobject), string);
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_object_properties_from_string_for_ustruct",
            [this](const uint64_t& value_ptr, const uint64_t& ustruct, const std::string& string) -> void {
                Unreal::setObjectPropertiesFromString(reinterpret_cast<void*>(value_ptr), reinterpret_cast<UStruct*>(ustruct), string);
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_property_by_name_on_uobject",
            [this](const uint64_t& uobject, const std::string& name) -> GameWorldServicePropertyDesc {
                return toPropertyDesc(Unreal::findPropertyByName(reinterpret_cast<UObject*>(uobject), name));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_property_by_name_on_ustruct",
            [this](const uint64_t& value_ptr, const uint64_t& ustruct, const std::string& name) -> GameWorldServicePropertyDesc {
                return toPropertyDesc(Unreal::findPropertyByName(reinterpret_cast<void*>(value_ptr), reinterpret_cast<UStruct*>(ustruct), name));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_property_value_as_string",
            [this](const GameWorldServicePropertyDesc& game_world_property_desc) -> std::string {
                return Unreal::getPropertyValueAsString(toPropertyDesc(game_world_property_desc));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_property_value_from_string",
            [this](const GameWorldServicePropertyDesc& game_world_property_desc, const std::string& string) -> void {
                Unreal::setPropertyValueFromString(toPropertyDesc(game_world_property_desc), string);
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_function_by_name",
            [this](const uint64_t& uclass, const std::string& name, const EIncludeSuperFlag::Type& include_super_flag) -> uint64_t {
                return reinterpret_cast<uint64_t>(Unreal::findFunctionByName(reinterpret_cast<UClass*>(uclass), name, include_super_flag));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "call_function",
            [this](const uint64_t& uobject, const uint64_t& ufunction, const std::map<std::string, std::string>& args) -> std::map<std::string, std::string> {
                return Unreal::callFunction(reinterpret_cast<UObject*>(uobject), reinterpret_cast<UFunction*>(ufunction), args);
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_special_struct_by_name",
            [this](const std::string& name) -> uint64_t {
                return reinterpret_cast<uint64_t>(Unreal::findSpecialStructByName(name));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "has_stable_name",
            [this](const uint64_t& actor) -> bool {
                return Unreal::hasStableName(reinterpret_cast<AActor*>(actor));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_stable_name_for_actor",
            [this](const uint64_t& actor) -> std::string {
                return Unreal::getStableName(reinterpret_cast<AActor*>(actor));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_stable_name_for_actor_component",
            [this](const uint64_t& actor_component, const bool& include_actor_name) -> std::string {
                return Unreal::getStableName(reinterpret_cast<UActorComponent*>(actor_component), include_actor_name);
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_stable_name_for_scene_component",
            [this](const uint64_t& scene_component, const bool& include_actor_name) -> std::string {
                return Unreal::getStableName(reinterpret_cast<USceneComponent*>(scene_component), include_actor_name);
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_actor_tags",
            [this](const uint64_t& actor) -> std::vector<std::string> {
                return Unreal::getTags(reinterpret_cast<AActor*>(actor));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_component_tags",
            [this](const uint64_t& component) -> std::vector<std::string> {
                return Unreal::getTags(reinterpret_cast<UActorComponent*>(component));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_class_from_instance",
            [this](const uint64_t& instance) -> uint64_t {
                return reinterpret_cast<uint64_t>(reinterpret_cast<UObject*>(instance)->GetClass());
            });
    }

    ~GameWorldService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);

private:

    template <typename TKey, typename TSrcValue>
    static std::map<TKey, uint64_t> toUint64(std::map<TKey, TSrcValue>&& input_map)
    {
        return Std::toMap<TKey, uint64_t>(
            input_map | 
            std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, reinterpret_cast<uint64_t>(value)); }));
    }

    template <typename TValueType>
    static std::vector<uint64_t> toUint64(const std::vector<TValueType>& src)
    {
        Std::reinterpretAsVectorOf<uint64_t>(src);
    }

    static GameWorldServicePropertyDesc toPropertyDesc(const Unreal::PropertyDesc& property_desc);
    static Unreal::PropertyDesc toPropertyDesc(const GameWorldServicePropertyDesc& game_world_property_desc);

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;

    UWorld* world_ = nullptr;
};
