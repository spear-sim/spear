//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // int32_t, int64_t, uint64_t

#include <map>
#include <string>
#include <vector>

#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"

//
// SpFuncDataBundle is intended as a high-level helper struct that can be used as the argument to, and the
// return value from, an SpFunc. We choose to make this a struct so it will be easier to add fields if
// necessary, without needing to explicitly update the signature of every SpFunc.
//

struct SpFuncDataBundle
{
    std::map<std::string, SpPackedArray> packed_arrays_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
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
