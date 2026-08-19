//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int32_t

#include <string>
#include <vector>

#include <HAL/Platform.h> // SPCORE_API

//
// SpConsoleMessages stores the messages returned by Console::flush(...), along with the number of
// messages that were evicted because the underlying ring buffer was full.
//

struct SpConsoleMessages
{
    std::vector<std::string> messages_;
    int32_t num_evicted_ = 0;
};

//
// Captures messages emitted via Unreal's logging system (UE_LOG, GLog->Log(...), etc.) into an in-memory
// ring buffer, so they can be retrieved later, e.g., to include in an error response sent back to a client
// that has no other way of seeing the Unreal log.
//

class SPCORE_API Console
{
public:
    Console() = delete;
    ~Console() = delete;

    static void requestInitialize();
    static void terminate();
    static bool isInitialized();

    static SpConsoleMessages flush();
};
