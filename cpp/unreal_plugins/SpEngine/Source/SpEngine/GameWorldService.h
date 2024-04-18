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
#include "SpCore/Unreal.h"
#include "SpEngine/EntryPointBinder.h"

class GameWorldService {
public:
    GameWorldService() = delete;
    GameWorldService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &GameWorldService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &GameWorldService::worldCleanupHandler);

        using TMapStringToString = std::map<std::string, std::string>;
        using TMapStringToInt = std::map<std::string, uint64_t>;

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_game_paused", [this](const bool& paused) -> void {
            SP_ASSERT(world_);
            UGameplayStatics::SetGamePaused(world_, paused);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "open_level", [this](const std::string& level_name) -> void {
            SP_ASSERT(world_);
            SP_LOG("Opening level: ", level_name);
            UGameplayStatics::OpenLevel(world_, Unreal::toFName(level_name));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_world_name", [this]() -> std::string {
            SP_ASSERT(world_);
            return Unreal::toStdString(world_->GetName());
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors", [this]() -> std::vector<uint64_t> {
            return Std::reinterpretAsVectorOf<uint64_t>(Unreal::findActors(world_));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_as_map", [this]() -> TMapStringToInt {
            return getIntMapFrom(Unreal::findActorsAsMap(world_));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components", [this](const uint64_t& ptr) -> std::vector<uint64_t> {
            return Std::reinterpretAsVectorOf<uint64_t>(Unreal::getComponents(reinterpretAs<AActor*>(ptr)));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_as_map", [this](const uint64_t& ptr) -> TMapStringToInt {
            return getIntMapFrom(Unreal::getComponentsAsMap(reinterpretAs<AActor*>(ptr)));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components", [this](const uint64_t& ptr, const bool& include_all_descendants) -> std::vector<uint64_t> {
            return Std::reinterpretAsVectorOf<uint64_t>(Unreal::getChildrenComponents(reinterpretAs<USceneComponent*>(ptr), include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_as_map", [this](const uint64_t& ptr, const bool& include_all_descendants) -> TMapStringToInt {
            return getIntMapFrom(Unreal::getChildrenComponentsAsMap(reinterpretAs<USceneComponent*>(ptr), include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_object_properties_as_string", [this](const uint64_t& ptr) -> std::string {
            return Unreal::getObjectPropertiesAsString(reinterpretAs<UObject*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_object_properties_as_string", [this](const uint64_t& value_ptr, const uint64_t& ustruct_ptr) -> std::string {
            return Unreal::getObjectPropertiesAsString(reinterpretAs<void*>(value_ptr), reinterpretAs<UStruct*>(ustruct_ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_object_properties_from_string", [this](const uint64_t& ptr, const std::string& string) -> void {
            Unreal::setObjectPropertiesFromString(reinterpretAs<UObject*>(ptr), string);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_object_properties_from_string", [this](const uint64_t& value_ptr, const uint64_t& ustruct_ptr, const std::string& string) -> void {
            Unreal::setObjectPropertiesFromString(reinterpretAs<void*>(value_ptr), reinterpretAs<UStruct*>(ustruct_ptr), string);
        });

        //unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_function_by_name", [this](const uint64_t& uclass_ptr, const std::string& name) -> uint64_t {
        //    return Unreal::findFunctionByName(reinterpretAs<UClass*>(uclass_ptr), name, EIncludeSuperFlag::Type include_super_flag = EIncludeSuperFlag::IncludeSuper);
        //});

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "call_function", [this](const uint64_t& uobject_ptr, const uint64_t& ufunction_ptr) -> TMapStringToString {
            return Unreal::callFunction(reinterpretAs<UObject*>(uobject_ptr), reinterpretAs<UFunction*>(ufunction_ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "call_function", [this](const uint64_t& uobject_ptr, const uint64_t& ufunction_ptr, const TMapStringToString& args) -> TMapStringToString {
            return Unreal::callFunction(reinterpretAs<UObject*>(uobject_ptr), reinterpretAs<UFunction*>(ufunction_ptr), args);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "actor_has_stable_name", [this](const uint64_t& ptr) -> bool {
            return Unreal::hasStableName(reinterpretAs<AActor*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "component_has_stable_name", [this](const uint64_t& ptr) -> bool {
            return Unreal::hasStableName(reinterpretAs<UActorComponent*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_actor_stable_name", [this](const uint64_t& ptr) -> std::string {
            return Unreal::getStableName(reinterpretAs<AActor*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_stable_name_for_actor_component", [this](const uint64_t& ptr, const bool& include_actor_name) -> std::string {
            return Unreal::getStableName(reinterpretAs<UActorComponent*>(ptr), include_actor_name);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_stable_name_for_scene_component", [this](const uint64_t& ptr, const bool& include_actor_name) -> std::string {
            return Unreal::getStableName(reinterpretAs<USceneComponent*>(ptr), include_actor_name);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_stable_name", [this](const uint64_t& ptr, const std::string& stable_name) -> void {
            Unreal::setStableName(reinterpretAs<AActor*>(ptr), stable_name);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_actor_tags", [this](const uint64_t& ptr) -> std::vector<std::string> {
            return Unreal::getTags(reinterpretAs<AActor*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_component_tags", [this](const uint64_t& ptr) -> std::vector<std::string> {
            return Unreal::getTags(reinterpretAs<UActorComponent*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_special_struct_by_name", [this](const std::string& name) -> uint64_t {
            return reinterpretAs<uint64_t>(Unreal::findSpecialStructByName(name));
        });

    #if WITH_EDITOR // defined in an auto-generated header
        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "request_update_stable_name", [this](const uint64_t& ptr) -> void {
            Unreal::requestUpdateStableName(reinterpretAs<AActor*>(ptr));
        });
    #endif
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
    std::map<TKey, uint64_t> getIntMapFrom(std::map<TKey, TSrcValue>&& input_map)
    {
        return Std::toMap<TKey, uint64_t>(
            input_map | 
            std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, reinterpret_cast<uint64_t>(value)); }));
    }

    template <typename TReturn>
    TReturn reinterpretAs(const uint64_t& ptr_addr)
    {
        SP_ASSERT(ptr_addr);
        return reinterpret_cast<TReturn>(ptr_addr);
    }

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;

    UWorld* world_ = nullptr;
};
