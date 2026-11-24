//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr
#include <string>

#include "SpCore/UnrealUtils.h"

#include "SpCoreEditor/UnrealEditor.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class UnrealServiceEditor : public Service
{
public:
    UnrealServiceEditor() = delete;
    UnrealServiceEditor(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        //
        // Get editor subsystem, !WITH_EDITOR implementations in SpServices/UnrealService_0.h
        //

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("unreal_service", "get_editor_subsystem_by_class",
            [this](SpHandle& uclass) -> SpHandle {
                return UnrealUtils::toHandle(UnrealEditor::getEditorSubsystemByClass(UnrealUtils::toPtr<UClass>(uclass)));
            });
    }

    template <CUnrealEntryPointBinder TEntryPointBinder>
    static std::unique_ptr<UnrealServiceEditor> create(TEntryPointBinder* unreal_entry_point_binder);
};
