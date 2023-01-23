//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <algorithm>
#include <cstring>
#include <map>

class COREUTILS_API Std
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

    template <typename TDest, typename TSrc>
    static std::vector<TDest> reinterpret_as(const std::vector<TSrc>& src)
    {
        std::vector<TDest> dest;
        if (src.size() > 0) {
            size_t src_bytes = src.size() * sizeof(TSrc);
            ASSERT(src_bytes % sizeof(TDest) == 0);
            size_t dest_elements = src_bytes / sizeof(TDest);
            dest.resize(dest_elements);
            std::memcpy(&dest.at(0), &src.at(0), src_bytes);
        }
        return dest;
    }
};
