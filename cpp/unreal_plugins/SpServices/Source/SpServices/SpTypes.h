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
class UFunction;
class UStruct;
class UWorld;

//
// SpFuncSignatureTypeDesc and SpFuncSignatureDesc are helper structs for tracking the function signatures
// of RPC entry points.
//

struct SpFuncSignatureTypeDesc
{
    std::map<std::string, std::string> type_names_;
    std::map<std::string, std::string> const_strings_;
    std::map<std::string, std::string> ref_strings_;
};

struct SpFuncSignatureDesc
{
    std::string name_;
    std::vector<SpFuncSignatureTypeDesc> func_signature_;
    std::vector<int> func_signature_id_;
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
// SpStaticStructDesc is a helper struct that stores reflection metadata for Unreal types.
//

struct SpStaticStructDesc
{
    UStruct* static_struct_ = nullptr;
    std::string name_;
    std::map<std::string, UFunction*> ufunctions_;
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
