//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <HAL/Platform.h> // SPCORE_API

#include "SpCore/SpTypes.h"

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
