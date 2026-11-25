//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

//
// UnrealService_1.h uses a "#define private public" hack to access several private variables, and therefore
// it must only be included in cpp files, and must be included after all other headers.
//

#include <stdint.h> // int32_t, int64_t

#include <map>
#include <memory> // std::unique_ptr
#include <ranges> // std::views::transform
#include <string>
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/Engine.h>         // GEngine
#include <Engine/LocalPlayer.h>    // ULocalPlayer
#include <Engine/World.h>          // UWorld
#include <HAL/IConsoleManager.h>   // EConsoleVariableFlags, IConsoleVariable
#include <HAL/Platform.h>          // uint64
#include <Modules/ModuleManager.h> // FModuleManager
#include <UObject/Class.h>         // UClass
#include <UObject/NameTypes.h>     // FName
#include <UObject/Object.h>        // UObject
#include <UObject/ObjectMacros.h>  // EObjectFlags

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
            [this](std::string& cvar_name) -> SpHandle {
                return UnrealUtils::toHandle(IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr(cvar_name)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_bool",
            [this](SpHandle& cvar) -> bool {
                SP_ASSERT(cvar.ptr_);
                return UnrealUtils::toPtr<IConsoleVariable>(cvar)->GetBool();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_int",
            [this](SpHandle& cvar) -> int64_t {
                SP_ASSERT(cvar.ptr_);
                return UnrealUtils::toPtr<IConsoleVariable>(cvar)->GetInt();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_float",
            [this](SpHandle& cvar) -> float {
                SP_ASSERT(cvar.ptr_);
                return UnrealUtils::toPtr<IConsoleVariable>(cvar)->GetFloat();
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_console_variable_value_as_string",
            [this](SpHandle& cvar) -> std::string {
                SP_ASSERT(cvar.ptr_);
                return Unreal::toStdString(UnrealUtils::toPtr<IConsoleVariable>(cvar)->GetString());
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_bool",
            [this](SpHandle& cvar, bool& val, std::vector<std::string>& set_by_strings, std::string& tag) -> void {
                SP_ASSERT(cvar.ptr_);
                UnrealUtils::toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings), Unreal::createFName(tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_int",
            [this](SpHandle& cvar, int32_t& val, std::vector<std::string>& set_by_strings, std::string& tag) -> void {
                SP_ASSERT(cvar.ptr_);
                UnrealUtils::toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings), Unreal::createFName(tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_float",
            [this](SpHandle& cvar, float& val, std::vector<std::string>& set_by_strings, std::string& tag) -> void {
                SP_ASSERT(cvar.ptr_);
                UnrealUtils::toPtr<IConsoleVariable>(cvar)->Set(val, Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings), Unreal::createFName(tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_from_string",
            [this](SpHandle& cvar, std::string& val, std::vector<std::string>& set_by_strings, std::string& tag) -> void {
                SP_ASSERT(cvar.ptr_);
                UnrealUtils::toPtr<IConsoleVariable>(cvar)->Set(Unreal::toTCharPtr(val), Unreal::getCombinedEnumFlagValueFromStringsAs<EConsoleVariableFlags, ESpConsoleVariableFlags>(set_by_strings), Unreal::createFName(tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_bool",
            [this](SpHandle& cvar, bool& val, std::string& tag) -> void {
                SP_ASSERT(cvar.ptr_);
                UnrealUtils::toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(val, Unreal::createFName(tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_int",
            [this](SpHandle& cvar, int32_t& val, std::string& tag) -> void {
                SP_ASSERT(cvar.ptr_);
                UnrealUtils::toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(val, Unreal::createFName(tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_float",
            [this](SpHandle& cvar, float& val, std::string& tag) -> void {
                SP_ASSERT(cvar.ptr_);
                UnrealUtils::toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(val, Unreal::createFName(tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_console_variable_value_with_current_priority_from_string",
            [this](SpHandle& cvar, std::string& val, std::string& tag) -> void {
                SP_ASSERT(cvar.ptr_);
                UnrealUtils::toPtr<IConsoleVariable>(cvar)->SetWithCurrentPriority(Unreal::toTCharPtr(val), Unreal::createFName(tag));
            });

        //
        // Execute console command
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "execute_console_command",
            [this](SpHandle& world, std::string& command) -> void {
                SP_ASSERT(GEngine);
                UWorld* world_ptr = UnrealUtils::toPtr<UWorld>(world);
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

                    std::vector<IMessageLogListingPtr> listings = Unreal::toStdVector(message_log_module->MessageLogViewModel->GetLogListingViewModels());
                    SP_ASSERT(Std::all(
                        listings |
                        std::views::transform([](const IMessageLogListingPtr& listing) { return listing.IsValid(); })));
                    return Std::toVector<std::string>(
                        listings |
                        std::views::transform([](const IMessageLogListingPtr& listing) { return Unreal::getUnrealName(listing->GetName()); })); // GetName() returns const FName& which doesn't satisfy CNameIsFNameObject
                #else
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
                    TSharedPtr<FMessageLogListingViewModel> listing;
                    for (const IMessageLogListingPtr& candidate : message_log_module->MessageLogViewModel->GetLogListingViewModels()) {
                        SP_ASSERT(candidate);
                        if (Unreal::isNamed(candidate->GetName(), name)) { // GetName() returns const FName& which doesn't satisfy CNameIsFNameObject
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
                    return std::vector<std::string>();
                #endif
            });

        //
        // Stable name helper functions
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_stable_name_for_actor",
            [this](SpHandle& actor, bool& include_unreal_id) -> std::string { return UnrealUtils::getStableName(UnrealUtils::toPtr<AActor>(actor), include_unreal_id); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "can_set_stable_name_for_actor",
            [this](SpHandle& actor) -> bool { return UnrealUtils::canSetStableName(UnrealUtils::toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "set_stable_name_for_actor",
            [this](SpHandle& actor, std::string& stable_name) -> void { UnrealUtils::setStableName(UnrealUtils::toPtr<AActor>(actor), stable_name); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "try_set_stable_name_for_actor",
            [this](SpHandle& actor, std::string& stable_name) -> bool { return UnrealUtils::trySetStableName(UnrealUtils::toPtr<AActor>(actor), stable_name); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_stable_name_for_component",
            [this](SpHandle& actor_component, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::string {
                return UnrealUtils::getStableName(UnrealUtils::toPtr<UActorComponent>(actor_component), include_actor_stable_name, include_actor_unreal_id);
            });

        //
        // Get actor and component tags
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_actor_tags",
            [this](SpHandle& actor) -> std::vector<std::string> { return Unreal::getTags(UnrealUtils::toPtr<AActor>(actor)); });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_tags",
            [this](SpHandle& component) -> std::vector<std::string> { return Unreal::getTags(UnrealUtils::toPtr<UActorComponent>(component)); });

        //
        // Find actors unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors",
            [this](SpHandle& world) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActors(UnrealUtils::toPtr<UWorld>(world)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_as_map",
            [this](SpHandle& world, bool& include_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsAsMap(UnrealUtils::toPtr<UWorld>(world), include_unreal_id));
            });

        //
        // Get components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components",
            [this](SpHandle& actor) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponents(UnrealUtils::toPtr<AActor>(actor)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_as_map",
            [this](SpHandle& actor, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsAsMap(UnrealUtils::toPtr<AActor>(actor), include_actor_stable_name, include_actor_unreal_id));
            });

        //
        // Get children components unconditionally and return an std::vector or std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor",
            [this](SpHandle& parent, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponents(UnrealUtils::toPtr<AActor>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_as_map",
            [this](SpHandle& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsAsMap(UnrealUtils::toPtr<AActor>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component",
            [this](SpHandle& parent, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponents(UnrealUtils::toPtr<USceneComponent>(parent), include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_as_map",
            [this](SpHandle& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsAsMap(UnrealUtils::toPtr<USceneComponent>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        //
        // Find actors conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_name",
            [this](SpHandle& uclass, SpHandle& world, std::string& actor_name) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByName(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag",
            [this](SpHandle& uclass, SpHandle& world, std::string& tag) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByTag(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_any",
            [this](SpHandle& uclass, SpHandle& world, std::vector<std::string>& tags) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByTagsAny(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_all",
            [this](SpHandle& uclass, SpHandle& world, std::vector<std::string>& tags) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByTagsAll(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_class",
            [this](SpHandle& uclass, SpHandle& world) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world)));
            });

        //
        // Find actors conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_name_as_map",
            [this](SpHandle& uclass, SpHandle& world, std::string& actor_name, bool& include_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByNameAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), actor_name, include_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tag_as_map",
            [this](SpHandle& uclass, SpHandle& world, std::string& tag, bool& include_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByTagAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), tag, include_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_any_as_map",
            [this](SpHandle& uclass, SpHandle& world, std::vector<std::string>& tags, bool& include_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByTagsAnyAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), tags, include_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_tags_all_as_map",
            [this](SpHandle& uclass, SpHandle& world, std::vector<std::string>& tags, bool& include_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByTagsAllAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), tags, include_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actors_by_class_as_map",
            [this](SpHandle& uclass, SpHandle& world, bool& include_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::findActorsByClassAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), include_unreal_id));
            });

        //
        // Find actor conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_name",
            [this](SpHandle& uclass, SpHandle& world, std::string& actor_name) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::findActorByName(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), actor_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tag",
            [this](SpHandle& uclass, SpHandle& world, std::string& tag) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::findActorByTag(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), tag));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tags_any",
            [this](SpHandle& uclass, SpHandle& world, std::vector<std::string>& tags) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::findActorByTagsAny(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_tags_all",
            [this](SpHandle& uclass, SpHandle& world, std::vector<std::string>& tags) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::findActorByTagsAll(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world), tags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "find_actor_by_class",
            [this](SpHandle& uclass, SpHandle& world) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::findActorByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UWorld>(world)));
            });

        //
        // Get components conditionally and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_name",
            [this](SpHandle& uclass, SpHandle& actor, std::string& component_name, bool& include_from_child_actors) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByName(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_path",
            [this](SpHandle& uclass, SpHandle& actor, std::string& component_path, bool& include_from_child_actors) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByPath(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag",
            [this](SpHandle& uclass, SpHandle& actor, std::string& tag, bool& include_from_child_actors) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByTag(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_any",
            [this](SpHandle& uclass, SpHandle& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByTagsAny(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_all",
            [this](SpHandle& uclass, SpHandle& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByTagsAll(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_class",
            [this](SpHandle& uclass, SpHandle& actor, bool& include_from_child_actors) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), include_from_child_actors));
            });

        //
        // Get components conditionally and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_name_as_map",
            [this](SpHandle& uclass, SpHandle& actor, std::string& component_name, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByNameAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), component_name, include_from_child_actors, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_path_as_map",
            [this](SpHandle& uclass, SpHandle& actor, std::string& component_path, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByPathAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), component_path, include_from_child_actors, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tag_as_map",
            [this](SpHandle& uclass, SpHandle& actor, std::string& tag, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByTagAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), tag, include_from_child_actors, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_any_as_map",
            [this](SpHandle& uclass, SpHandle& actor, std::vector<std::string>& tags, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByTagsAnyAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), tags, include_from_child_actors, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_tags_all_as_map",
            [this](SpHandle& uclass, SpHandle& actor, std::vector<std::string>& tags, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByTagsAllAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), tags, include_from_child_actors, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_components_by_class_as_map",
            [this](SpHandle& uclass, SpHandle& actor, bool& include_from_child_actors, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getComponentsByClassAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), include_from_child_actors, include_actor_stable_name, include_actor_unreal_id));
            });

        //
        // Get component conditionally
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_name",
            [this](SpHandle& uclass, SpHandle& actor, std::string& component_name, bool& include_from_child_actors) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getComponentByName(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), component_name, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_path",
            [this](SpHandle& uclass, SpHandle& actor, std::string& component_path, bool& include_from_child_actors) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getComponentByPath(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), component_path, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tag",
            [this](SpHandle& uclass, SpHandle& actor, std::string& tag, bool& include_from_child_actors) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getComponentByTag(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), tag, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tags_any",
            [this](SpHandle& uclass, SpHandle& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getComponentByTagsAny(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_tags_all",
            [this](SpHandle& uclass, SpHandle& actor, std::vector<std::string>& tags, bool& include_from_child_actors) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getComponentByTagsAll(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), tags, include_from_child_actors));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_component_by_class",
            [this](SpHandle& uclass, SpHandle& actor, bool& include_from_child_actors) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getComponentByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(actor), include_from_child_actors));
            });

        //
        // Get children components conditionally from an actor and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_name",
            [this](SpHandle& uclass, SpHandle& parent, std::string& child_component_name, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByName(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tag",
            [this](SpHandle& uclass, SpHandle& parent, std::string& tag, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTag(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_any",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagsAny(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_all",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagsAll(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_class",
            [this](SpHandle& uclass, SpHandle& parent, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from an actor and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_name_as_map",
            [this](SpHandle& uclass, SpHandle& parent, std::string& child_component_name, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByNameAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), child_component_name, include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tag_as_map",
            [this](SpHandle& uclass, SpHandle& parent, std::string& tag, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), tag, include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_any_as_map",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagsAnyAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_tags_all_as_map",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagsAllAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_actor_by_class_as_map",
            [this](SpHandle& uclass, SpHandle& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByClassAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        //
        // Get child component conditionally from an actor
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_name",
            [this](SpHandle& uclass, SpHandle& parent, std::string& child_component_name, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByName(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_tag",
            [this](SpHandle& uclass, SpHandle& parent, std::string& tag, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByTag(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_tags_any",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByTagsAny(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_tags_all",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByTagsAll(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_actor_by_class",
            [this](SpHandle& uclass, SpHandle& parent, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::vector
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_name",
            [this](SpHandle& uclass, SpHandle& parent, std::string& child_component_name, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByName(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tag",
            [this](SpHandle& uclass, SpHandle& parent, std::string& tag, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTag(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_any",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagsAny(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_all",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagsAll(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_class",
            [this](SpHandle& uclass, SpHandle& parent, bool& include_all_descendants) -> std::vector<SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Get children components conditionally from a scene component and return an std::map
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_name_as_map",
            [this](SpHandle& uclass, SpHandle& parent, std::string& child_component_name, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByNameAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), child_component_name, include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tag_as_map",
            [this](SpHandle& uclass, SpHandle& parent, std::string& tag, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), tag, include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_any_as_map",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagsAnyAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_tags_all_as_map",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByTagsAllAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), tags, include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_children_components_for_scene_component_by_class_as_map",
            [this](SpHandle& uclass, SpHandle& parent, bool& include_all_descendants, bool& include_actor_stable_name, bool& include_actor_unreal_id) -> std::map<std::string, SpHandle> {
                return UnrealUtils::toHandle(UnrealUtils::getChildrenComponentsByClassAsMap(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), include_all_descendants, include_actor_stable_name, include_actor_unreal_id));
            });

        //
        // Get child component conditionally from a scene component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_name",
            [this](SpHandle& uclass, SpHandle& parent, std::string& child_component_name, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByName(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), child_component_name, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_tag",
            [this](SpHandle& uclass, SpHandle& parent, std::string& tag, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByTag(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), tag, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_tags_any",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByTagsAny(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_tags_all",
            [this](SpHandle& uclass, SpHandle& parent, std::vector<std::string>& tags, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByTagsAll(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_child_component_for_scene_component_by_class",
            [this](SpHandle& uclass, SpHandle& parent, bool& include_all_descendants) -> SpHandle {
                return UnrealUtils::toHandle(UnrealUtils::getChildComponentByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(parent), include_all_descendants));
            });

        //
        // Create component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_component_outside_owner_constructor_by_class",
            [this](SpHandle& uclass, SpHandle& owner, std::string& component_name, std::vector<std::string>& object_flag_strings) -> SpHandle {
                EObjectFlags object_flags = Unreal::getCombinedEnumFlagValueFromStringsAs<EObjectFlags, ESpObjectFlags>(object_flag_strings);
                return UnrealUtils::toHandle(UnrealUtils::createComponentOutsideOwnerConstructorByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(owner), component_name, object_flags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_for_actor_by_class",
            [this](SpHandle& uclass, SpHandle& owner, std::string& scene_component_name, std::vector<std::string>& object_flag_strings) -> SpHandle {
                EObjectFlags object_flags = Unreal::getCombinedEnumFlagValueFromStringsAs<EObjectFlags, ESpObjectFlags>(object_flag_strings);
                return UnrealUtils::toHandle(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<AActor>(owner), scene_component_name, object_flags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_for_object_by_class",
            [this](SpHandle& uclass, SpHandle& owner, SpHandle& parent, std::string& scene_component_name, std::vector<std::string>& object_flag_strings) -> SpHandle {
                EObjectFlags object_flags = Unreal::getCombinedEnumFlagValueFromStringsAs<EObjectFlags, ESpObjectFlags>(object_flag_strings);
                return UnrealUtils::toHandle(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<UObject>(owner), UnrealUtils::toPtr<USceneComponent>(parent), scene_component_name, object_flags));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "create_scene_component_outside_owner_constructor_for_scene_component_by_class",
            [this](SpHandle& uclass, SpHandle& owner, std::string& scene_component_name, std::vector<std::string>& object_flag_strings) -> SpHandle {
                EObjectFlags object_flags = Unreal::getCombinedEnumFlagValueFromStringsAs<EObjectFlags, ESpObjectFlags>(object_flag_strings);
                return UnrealUtils::toHandle(UnrealUtils::createSceneComponentOutsideOwnerConstructorByClass(UnrealUtils::toPtr<UClass>(uclass), UnrealUtils::toPtr<USceneComponent>(owner), scene_component_name, object_flags));
            });

        //
        // Destroy component
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "destroy_component_outside_owner_constructor",
            [this](SpHandle& component, bool& promote_children) -> void {
                SP_ASSERT(component.ptr_);
                UnrealUtils::destroyComponentOutsideOwnerConstructor(UnrealUtils::toPtr<UActorComponent>(component), promote_children);
            });
    }

    template <CUnrealEntryPointBinder TEntryPointBinder>
    static std::unique_ptr<UnrealService_1> create(TEntryPointBinder* unreal_entry_point_binder);
};
