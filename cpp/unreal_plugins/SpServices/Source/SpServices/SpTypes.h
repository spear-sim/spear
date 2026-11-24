//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t

#include <map>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle

#include "SpCore/UnrealUtils.h"

//
// SpFuncSignatureDesc is a helper struct for tracking the function signatures of RPC entry points.
//

struct SpFuncSignatureDesc // exposed to Python via SpServices/MsgpackAdaptors.h
{
    std::string name_;
    std::vector<int> type_ids_;
    std::vector<std::string> type_names_;
};

//
// SpFuture is a weakly typed wrapper around an std::future<T>, with sufficient metadata to ensure that it is
// being cast correctly when casting back to a particular type.
//

struct SpFuture // exposed to Python via SpServices/MsgpackAdaptors.h
{
    void* future_ptr_ = nullptr;
    std::string type_id_;
};

//
// SpFunctionDesc stores metadata about a UFunction.
//

struct SpFunctionDesc // exposed to Python via SpServices/MsgpackAdaptors.h
{
    SpHandle function_;     // SpHandle needed so SpFunctionDesc can be serialized safely on the worker thread
    std::string function_name_;
    SpHandle static_class_; // SpHandle needed so SpFunctionDesc can be serialized safely on the worker thread
    std::string static_class_name_;
};

//
// SpStaticStructDesc stores reflection metadata for a UScriptStruct.
//

struct SpStaticStructDesc // exposed to Python via SpServices/MsgpackAdaptors.h
{
    SpHandle static_struct_; // SpHandle needed so SpStaticStructDesc can be serialized safely on the worker thread
    std::string name_;
};

//
// SpStaticClassDesc stores reflection metadata for a UClass.
//

struct SpStaticClassDesc // exposed to Python via SpServices/MsgpackAdaptors.h
{
    SpHandle static_class_;                 // SpHandle needed so SpStaticClassDesc can be serialized safely on the worker thread
    std::string name_;
    std::vector<SpHandle> derived_classes_; // SpHandle needed so SpStaticClassDesc can be serialized safely on the worker thread
    std::vector<std::string> derived_class_names_;
    std::map<std::string, SpFunctionDesc> function_descs_;
};

//
// SpWorldDesc stores the state of a tracked UWorld.
//

struct SpWorldDesc // exposed to Python via SpServices/MsgpackAdaptors.h
{
    SpHandle world_; // SpHandle needed so SpWorldDesc can be serialized safely on the worker thread
    int64_t world_id_ = -1;
    bool is_editor_world_ = false;
    bool is_game_world_ = false;
    bool is_playing_ = false;
    FDelegateHandle world_begin_play_handle_;
};
