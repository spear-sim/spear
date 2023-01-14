//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <algorithm>
#include <map>

class COREUTILS_API StdUtils
{
public:
    template <typename TContainer, typename TKey>
    static bool contains(const TContainer& container, const TKey& key)
    {
        return std::find(container.begin(), container.end(), key) != container.end();
    }

    template <typename TContainer, typename TKey>
    static bool containsKey(const TContainer& container, const TKey& key)
    {
        return container.count(key) > 0;
    }
};
