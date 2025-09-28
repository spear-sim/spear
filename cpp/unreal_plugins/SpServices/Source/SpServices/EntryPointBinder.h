//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "SpCore/Std.h"

enum class UnrealEntryPointBindFlags
{
    DoNotBind = 0,
    CallSync  = 1 << 0,
    CallAsync = 1 << 1,
    SendAsync = 1 << 2
};
SP_DECLARE_ENUM_FLAG_OPERATORS(UnrealEntryPointBindFlags);

template <typename TEntryPointBinder>
concept CEntryPointBinder =
    requires(TEntryPointBinder entry_point_binder) {
        { entry_point_binder.bind("", []() -> void {}) } -> std::same_as<void>;
    };

template <typename TUnrealEntryPointBinder>
concept CUnrealEntryPointBinder =
    requires(TUnrealEntryPointBinder unreal_entry_point_binder) {
        { unreal_entry_point_binder.bindFuncToExecuteOnWorkerThread("", "", []() -> void {}) } -> std::same_as<void>;
        { unreal_entry_point_binder.bindFuncToExecuteOnGameThread("", "", []() -> void {}) } -> std::same_as<void>;
        { unreal_entry_point_binder.bindFuncToExecuteOnGameThread("", "", UnrealEntryPointBindFlags::DoNotBind, []() -> void {}) } -> std::same_as<void>;
    };
