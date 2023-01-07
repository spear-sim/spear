//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <algorithm>

class COREUTILS_API StdUtils
{
public:
    template <typename TContainer, typename TKey>
    static bool contains(const TContainer& container, const TKey& key) {
        return std::find(container.begin(), container.end(), key) != container.end();
    }
};
