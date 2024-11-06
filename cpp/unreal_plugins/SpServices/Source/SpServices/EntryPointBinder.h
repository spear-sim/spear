//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

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
    };
