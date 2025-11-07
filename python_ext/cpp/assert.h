//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <iostream>
#include <sstream>
#include <stdexcept> // std::runtime_error

#define SP_ASSERT(expression)                                                                     \
    do {                                                                                          \
        if (!(expression)) {                                                                      \
            std::stringstream ss;                                                                 \
            ss << "Assert failed: " << #expression << " (" << __FILE__ << ":" << __LINE__ << ")"; \
            std::cout << ss.str() << std::endl;                                                   \
            throw std::runtime_error(ss.str());                                                   \
        }                                                                                         \
    }                                                                                             \
    while (false)
