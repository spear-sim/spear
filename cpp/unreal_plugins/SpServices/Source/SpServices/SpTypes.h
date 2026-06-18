//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle

class FProperty;
class UClass;
class UFunction;
class UStruct;
class UWorld;

//
// SpFuncSignatureDesc is a helper structs for tracking the function signatures of RPC entry points.
//

struct SpFuncSignatureDesc
{
    std::string name_;
    std::vector<int> type_ids_;
    std::vector<std::string> type_names_;
};

//
// SpFuture is a weakly typed wrapper around an std::future<T>, with sufficient metadata to ensure that it is
// being cast correctly when casting back to a particular type.
//

struct SpFuture
{
    void* future_ptr_ = nullptr;
    std::string type_id_;
};

//
// SpFunctionDesc stores metadata about a UFunction.
//

struct SpFunctionDesc
{
    UFunction* function_ = nullptr;
    std::string function_name_;
    UClass* static_class_ = nullptr;
    std::string static_class_name_;
};

//
// SpStaticStructDesc stores reflection metadata for a UScriptStruct.
//

struct SpStaticStructDesc
{
    UStruct* static_struct_ = nullptr;
    std::string name_;
};

//
// SpStaticClassDesc stores reflection metadata for a UClass.
//

struct SpStaticClassDesc
{
    UClass* static_class_ = nullptr;
    std::string name_;
    std::vector<UClass*> derived_classes_;
    std::vector<std::string> derived_class_names_;
    std::map<std::string, SpFunctionDesc> function_descs_;
};

//
// SpWorldDesc stores the state of a tracked UWorld.
//

struct SpWorldDesc
{
    UWorld* world_ = nullptr;
    int64_t world_id_ = -1;
    bool is_editor_world_ = false;
    bool is_game_world_ = false;
    bool is_playing_ = false;
    FDelegateHandle world_begin_play_handle_;
};
