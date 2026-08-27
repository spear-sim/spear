//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

//
// UnrealService_1.h uses a "#define public private" hack to access several private variables, and therefore
// it must only be included in cpp files, and must be included after all other headers.
//

#include <stdint.h> // uint64_t

#include <map>
#include <memory> // std::unique_ptr
#include <ranges> // std::views::transform
#include <string>
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/Engine.h>         // GEngine
#include <Engine/LocalPlayer.h>    // ULocalPlayer
#include <HAL/IConsoleManager.h>   // EConsoleVariableFlags, IConsoleVariable
#include <HAL/Platform.h>          // uint64
#include <Modules/ModuleManager.h> // FModuleManager
#include <UObject/Class.h>         // UClass
#include <UObject/NameTypes.h>     // FName
#include <UObject/Object.h>        // UObject

// MessageLog is a Developer-category module and is therefore not available in Shipping builds.
#if WITH_UNREAL_DEVELOPER_TOOLS

    #include <IMessageLogListing.h>       // IMessageLogListing, IMessageLogListingPtr
    #include <Logging/TokenizedMessage.h> // FTokenizedMessage

    // private headers need custom include paths in SpModuleRules.Build.cs
    #include <Model/MessageLogListingModel.h> // FMessageLogListingModel, MessageContainer

#endif

#include "SpCore/Assert.h"
#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/OutputLog.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"
#include "SpServices/SpTypes.h"
#include "SpServices/UnrealServiceTypes.h"

// MessageLog is a Developer-category module and is therefore not available in Shipping builds.
#if WITH_UNREAL_DEVELOPER_TOOLS

    //
    // HACK: FMessageLogModule::MessageLogViewModel and FMessageLogListingViewModel::MessageLogListingModel are
    // both private members, and there is no public API for enumerating all registered message log listings, or
    // for reading all (as opposed to filtered) messages across all pages of a listing. We override private
    // visibility in the headers below, but only ever reach into these two members directly. Reading a private
    // member needs no exported symbol, since it's just a memory offset, not a function call, so this links
    // correctly.
    //
    // Every method we call from the headers below is either inline (FMessageLogViewModel::GetLogListingViewModels())
    // or a member of a class that is properly exported from the MessageLog module (FMessageLogListingViewModel and
    // FMessageLogListingModel are both MESSAGELOG_API), so this links correctly. FMessageLogViewModel and
    // FMessageLogModule themselves are not exported, so we deliberately avoid calling any of their non-inline
    // methods (e.g., FindLogListingViewModel(...)). We must include Presentation/MessageLogListingViewModel.h
    // under this #define too (not just MessageLogModule.h), because Presentation/MessageLogViewModel.h
    // transitively includes it. If that transitive include happens first, outside this block, its header guard
    // would make our own include below a no-op, leaving MessageLogListingModel private after all.
    //

    #pragma push_macro("private")
    #undef private
    #define private public
        #include <Presentation/MessageLogListingViewModel.h> // FMessageLogListingViewModel
        #include <Presentation/MessageLogViewModel.h>        // FMessageLogViewModel
        #include <MessageLogModule.h>                        // FMessageLogModule
    #pragma pop_macro("private")

#endif

class UnrealService_1 : public Service
{
public:
    UnrealService_1() = delete;
    UnrealService_1(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        //
        // Find, get, and set console variable
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_console_variable_by_name",
            [this](std::string& cvar_name) -> uint64_t {
                return toUInt64(IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr(cvar_name)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_bool",
            [this](uint64_t& cvar) -> bool {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetBool();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_int",
            [this](uint64_t& cvar) -> int64_t {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetInt();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_float",
            [this](uint64_t& cvar) -> float {
                SP_ASSERT(cvar);
                return toPtr<IConsoleVariable>(cvar)->GetFloat();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_string",
            [this](uint64_t& cvar) -> std::string {
                SP_ASSERT(cvar);
                return Unreal::toStdString(toPtr<IConsoleVariable>(cvar)->GetString());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_bool",
            [this](uint64_t& cvar, bool& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_int",
            [this](uint64_t& cvar, int32_t& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_float",
            [this](uint64_t& cvar, float& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_string",
            [this](uint64_t& cvar, std::string& val, std::vector<std::string>& set_by_strings) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->Set(Unreal::toTCharPtr(val), Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_bool",
            [this](uint64_t& cvar, bool& val) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(val);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_int",
            [this](uint64_t& cvar, int32_t& val) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(val);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_float",
            [this](uint64_t& cvar, float& val) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(val);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_string",
            [this](uint64_t& cvar, std::string& val) -> void {
                SP_ASSERT(cvar);
                toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(Unreal::toTCharPtr(val));
            });

        //
        // Execute console command
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "execute_console_command",
            [this](uint64_t& world, std::string& command) -> void {
                SP_ASSERT(GEngine);
                UWorld* world_ptr = toPtr<UWorld>(world);
                ULocalPlayer* local_player = GEngine->GetFirstGamePlayer(world_ptr);

                bool handled = false;
                if (!handled && local_player) {
                    handled = local_player->Exec(world_ptr, Unreal::toTCharPtr(command), *GLog);
                } if (!handled) {
                    handled = GEngine->Exec(world_ptr, Unreal::toTCharPtr(command));
                } if (!handled) {
                    SP_LOG("WARNING: \"", command, "\" command not handled.");
                }
            });

        //
        // Flush output log messages
        //

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("unreal_service", "flush_output_log_messages",
            [this]() -> SpOutputLogMessages { return OutputLog::flush(); });

        //
        // Get message log names and messages
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_message_log_names",
            [this]() -> std::vector<std::string> {
                // Not available in Shipping builds, since MessageLog is a Developer-category module -- see
                // the WITH_UNREAL_DEVELOPER_TOOLS include guards above.
                #if WITH_UNREAL_DEVELOPER_TOOLS
                    SP_ASSERT_MODULE_LOADED("MessageLog");
                    FMessageLogModule* message_log_module = FModuleManager::Get().GetModulePtr<FMessageLogModule>("MessageLog");
                    SP_ASSERT(message_log_module);
                    SP_ASSERT(message_log_module->MessageLogViewModel);

                    return Std::toVector<std::string>(
                        Unreal::toStdVector(message_log_module->MessageLogViewModel->GetLogListingViewModels()) |
                        std::views::transform([](const IMessageLogListingPtr& listing) { SP_ASSERT(listing); return Unreal::toStdString(listing->GetName()); }));
                #else
                    SP_ASSERT(false);
                    return std::vector<std::string>();
                #endif
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_message_log_messages",
            [this](std::string& name) -> std::vector<std::string> {
                // Not available in Shipping builds, since MessageLog is a Developer-category module -- see
                // the WITH_UNREAL_DEVELOPER_TOOLS include guards above.
                #if WITH_UNREAL_DEVELOPER_TOOLS
                    SP_ASSERT_MODULE_LOADED("MessageLog");
                    FMessageLogModule* message_log_module = FModuleManager::Get().GetModulePtr<FMessageLogModule>("MessageLog");
                    SP_ASSERT(message_log_module);
                    SP_ASSERT(message_log_module->MessageLogViewModel);

                    // FMessageLogViewModel::FindLogListingViewModel(...) and FMessageLogModel::GetLogListingModel(...)
                    // are not exported from the MessageLog module (FMessageLogViewModel and FMessageLogModule have
                    // no MESSAGELOG_API), so we can't call them directly across the module boundary. We instead
                    // linearly search the (inline, and therefore always linkable) GetLogListingViewModels() array
                    // for the matching name.
                    FName log_name = Unreal::toFName(name);
                    TSharedPtr<FMessageLogListingViewModel> listing;
                    for (const IMessageLogListingPtr& candidate : message_log_module->MessageLogViewModel->GetLogListingViewModels()) {
                        SP_ASSERT(candidate);
                        if (candidate->GetName() == log_name) {
                            listing = StaticCastSharedPtr<FMessageLogListingViewModel>(candidate);
                            break;
                        }
                    }
                    SP_ASSERT(listing);
                    SP_ASSERT(listing->MessageLogListingModel);

                    // Deliberately read from FMessageLogListingModel rather than
                    // FMessageLogListingViewModel::GetFilteredMessages(...), so we return every message across
                    // every page, regardless of any severity/search filtering that might be active in the UI.
                    TSharedPtr<FMessageLogListingModel> listing_model = listing->MessageLogListingModel;

                    std::vector<std::string> messages;
                    for (uint32 page_index = 0; page_index < listing_model->NumPages(); page_index++) {
                        for (MessageContainer::TConstIterator it = listing_model->GetMessageIterator(page_index); it; ++it) {
                            messages.push_back(Unreal::toStdString((*it)->ToText().ToString()));
                        }
                    }

                    return messages;
                #else
                    SP_ASSERT(false);
                    return std::vector<std::string>();
                #endif
            });

        //
        // Stable name helper functions
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_stable_name_for_actor",
            [this](uint64_t& actor, bool& include_unreal_name) -> std::string { return UnrealUtils::getStableName(toPtr<AActor>(actor), include_unreal_name); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_stable_name_for_actor",
            [this](uint64_t& actor, std::string& stable_name) -> void { UnrealUtils::setStableName(toPtr<AActor>(actor), stable_name); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_stable_name_for_component",
            [this](uint64_t& actor_component, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::string {
                return UnrealUtils::getStableName(toPtr<UActorComponent>(actor_component), include_actor_stable_name, include_actor_unreal_name);
            });

        //
        // Get actor and component tags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_actor_tags",
            [this](uint64_t& actor) -> std::vector<std::string> { return Unreal::getTags(toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_tags",
            [this](uint64_t& component) -> std::vector<std::string> { return Unreal::getTags(toPtr<UActorComponent>(component)); });

        //
        // Find actors unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors",
            [this](uint64_t& world) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActors(toPtr<UWorld>(world)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_as_map",
            [this](uint64_t& world, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsAsMap(toPtr<UWorld>(world), include_unreal_name));
            });

        //
        // Get components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components",
            [this](uint64_t& actor) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponents(toPtr<AActor>(actor)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_as_map",
            [this](uint64_t& actor, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsAsMap(toPtr<AActor>(actor), include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Get children components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponents(toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_as_map",
            [this](uint64_t& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsAsMap(toPtr<AActor>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component",
            [this](uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponents(toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_as_map",
            [this](uint64_t& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsAsMap(toPtr<USceneComponent>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Find actors conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_name",
            [this](uint64_t& uclass, uint64_t& world, std::string& actor_name) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByName(toPtr<UClass>(uclass), toPtr<UWorld>(world), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag",
            [this](uint64_t& uclass, uint64_t& world, std::string& tag) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTag(toPtr<UClass>(uclass), toPtr<UWorld>(world), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_any",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAny(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_all",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAll(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_class",
            [this](uint64_t& uclass, uint64_t& world) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::findActorsByClass(toPtr<UClass>(uclass), toPtr<UWorld>(world)));
            });

        //
        // Find actors conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& world, std::string& actor_name, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByNameAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), actor_name, include_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& world, std::string& tag, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), tag, include_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags, include_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags, include_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& world, bool& include_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::findActorsByClassAsMap(toPtr<UClass>(uclass), toPtr<UWorld>(world), include_unreal_name));
            });

        //
        // Find actor conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_name",
            [this](uint64_t& uclass, uint64_t& world, std::string& actor_name) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByName(toPtr<UClass>(uclass), toPtr<UWorld>(world), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tag",
            [this](uint64_t& uclass, uint64_t& world, std::string& tag) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByTag(toPtr<UClass>(uclass), toPtr<UWorld>(world), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tags_any",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByTagsAny(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tags_all",
            [this](uint64_t& uclass, uint64_t& world, std::vector<std::string>& tags) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByTagsAll(toPtr<UClass>(uclass), toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_class",
            [this](uint64_t& uclass, uint64_t& world) -> uint64_t {
                return toUInt64(UnrealUtils::findActorByClass(toPtr<UClass>(uclass), toPtr<UWorld>(world)));
            });

        //
        // Get components conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_name",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_name, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByName(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_path",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_path, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByPath(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag",
            [this](uint64_t& uclass, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTag(toPtr<UClass>(uclass), toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_any",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_all",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_class",
            [this](uint64_t& uclass, uint64_t& actor, bool& include_from_child_actors) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByClass(toPtr<UClass>(uclass), toPtr<AActor>(actor), include_from_child_actors));
            });

        //
        // Get components conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_name, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByNameAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_name, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_path_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_path, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByPathAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_path, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::string& tag, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), tag, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& actor, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getComponentsByClassAsMap(toPtr<UClass>(uclass), toPtr<AActor>(actor), include_from_child_actors, include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Get component conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_name",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_name, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByName(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_path",
            [this](uint64_t& uclass, uint64_t& actor, std::string& component_path, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByPath(toPtr<UClass>(uclass), toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tag",
            [this](uint64_t& uclass, uint64_t& actor, std::string& tag, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByTag(toPtr<UClass>(uclass), toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tags_any",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tags_all",
            [this](uint64_t& uclass, uint64_t& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_class",
            [this](uint64_t& uclass, uint64_t& actor, bool& include_from_child_actors) -> uint64_t {
                return toUInt64(UnrealUtils::getComponentByClass(toPtr<UClass>(uclass), toPtr<AActor>(actor), include_from_child_actors));
            });

        //
        // Get children components conditionally from an actor and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByName(toPtr<UClass>(uclass), toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTag(toPtr<UClass>(uclass), toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClass(toPtr<UClass>(uclass), toPtr<AActor>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from an actor and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByNameAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), child_component_name, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), tag, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClassAsMap(toPtr<UClass>(uclass), toPtr<AActor>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Get child component conditionally from an actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByName(toPtr<UClass>(uclass), toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTag(toPtr<UClass>(uclass), toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAny(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAll(toPtr<UClass>(uclass), toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByClass(toPtr<UClass>(uclass), toPtr<AActor>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByName(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTag(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAny(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAll(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> std::vector<uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClass(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_name_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByNameAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), child_component_name, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tag_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tag, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_any_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAnyAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_all_as_map",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByTagsAllAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_class_as_map",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_name) -> std::map<std::string, uint64_t> {
                return toUInt64(UnrealUtils::getChildrenComponentsByClassAsMap(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_name));
            });

        //
        // Get child component conditionally from a scene component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_name",
            [this](uint64_t& uclass, uint64_t& parent, std::string& child_component_name, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByName(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_tag",
            [this](uint64_t& uclass, uint64_t& parent, std::string& tag, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTag(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_tags_any",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAny(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_tags_all",
            [this](uint64_t& uclass, uint64_t& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByTagsAll(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_class",
            [this](uint64_t& uclass, uint64_t& parent, bool& include_all_descendants) -> uint64_t {
                return toUInt64(UnrealUtils::getChildComponentByClass(toPtr<UClass>(uclass), toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Create component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_component_outside_owner_constructor_by_class",
            [this](uint64_t& uclass, uint64_t& owner, std::string& component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<AActor>(owner), component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_for_actor_by_class",
            [this](uint64_t& uclass, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<AActor>(owner), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_for_object_by_class",
            [this](uint64_t& uclass, uint64_t& owner, uint64_t& parent, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<UObject>(owner), toPtr<USceneComponent>(parent), scene_component_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_for_scene_component_by_class",
            [this](uint64_t& uclass, uint64_t& owner, std::string& scene_component_name) -> uint64_t {
                return toUInt64(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(toPtr<UClass>(uclass), toPtr<USceneComponent>(owner), scene_component_name));
            });

        //
        // Destroy component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "destroy_component_outside_owner_constructor",
            [this](uint64_t& component, bool& promote_children) -> void {
                SP_ASSERT(component);
                UnrealUtils::destroyComponentOutsideOwnerConstructor(toPtr<UActorComponent>(component), promote_children);
            });
    }

    template <CUnrealEntryPointBinder TEntryPointBinder>
    static std::unique_ptr<UnrealService_1> create(TEntryPointBinder* unreal_entry_point_binder);
};
