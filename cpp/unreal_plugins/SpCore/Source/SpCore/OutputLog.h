//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int32_t

#include <string>
#include <vector>

#include <HAL/Platform.h> // SPCORE_API

struct SpOutputLogMessages
{
    std::vector<std::string> messages_;
    int32_t num_evicted_ = 0;
};

class SPCORE_API OutputLog
{
public:
    OutputLog() = delete;
    ~OutputLog() = delete;

    static void requestInitialize();
    static void terminate();
    static bool isInitialized();

    static SpOutputLogMessages flush();
};
