//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <atomic>
#include <string>

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class InitializeEditorWorldService : public Service {
public:
    InitializeEditorWorldService() = delete;
    InitializeEditorWorldService(CUnrealEntryPointBinder auto* unreal_entry_point_binder, Service::WorldFilter* world_filter) : Service("InitializeEditorWorldService", world_filter)
    {
        std::string service_name = getWorldTypeName() + ".initialize_editor_world_service";

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread(service_name, "is_initialized", [this]() -> bool {
            return initialized_;
        });
    }

protected:
    void postWorldInitialization(UWorld* world, const UWorld::InitializationValues initialization_values) override
    {
        Service::postWorldInitialization(world, initialization_values);
        initialized_ = true;
    }

    void worldCleanup(UWorld* world, bool session_ended, bool cleanup_resources) override
    {
        initialized_ = false;
        Service::worldCleanup(world, session_ended, cleanup_resources);
    }

private:
    // Initialization state
    std::atomic<bool> initialized_ = false;
};
