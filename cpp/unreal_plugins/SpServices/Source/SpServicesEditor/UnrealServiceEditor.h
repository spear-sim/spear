//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // uint64_t

#include <string>

#include "SpCore/UnrealClassRegistrar.h"

#include "SpCoreEditor/UnrealEditor.h"
#include "SpCoreEditor/UnrealClassRegistrarEditor.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class UnrealServiceEditor : public Service
{
public:
    UnrealServiceEditor() = delete;
    UnrealServiceEditor(CUnrealEntryPointBinder auto* unreal_entry_point_binder, Service::WorldFilter* world_filter) : Service("UnrealServiceEditor", world_filter)
    {
        std::string service_name = getWorldTypeName() + ".unreal_service";

        //
        // Get editor subsystem, !WITH_EDITOR implementations in SpServices/UnrealService.h
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_editor_subsystem_by_type",
            [this](std::string& class_name) -> uint64_t {
                return toUInt64(UnrealClassRegistrarEditor::getEditorSubsystemByType(class_name));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread(service_name, "get_editor_subsystem_by_class",
            [this](uint64_t& uclass) -> uint64_t {
                return toUInt64(UnrealEditor::getEditorSubsystemByClass(toPtr<UClass>(uclass))); // UnrealClassRegistrarEditor not needed because UnrealEditor::getEditorSubsystemBase(...) has no template parameters
            });
    }
};
