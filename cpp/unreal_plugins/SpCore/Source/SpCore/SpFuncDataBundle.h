//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>

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
