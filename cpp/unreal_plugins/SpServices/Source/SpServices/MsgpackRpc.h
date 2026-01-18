//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

// msgpack and rpclib headers
#include "SpServices/Rpclib.h"

// adaptor helper functions
#include "SpServices/MsgpackUtils.h"

// adaptors (potentially needed at any call site that calls rpc::server::bind(...))
#include "SpServices/MsgpackAdaptors.h"
