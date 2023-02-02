//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <algorithm>
#include <cstring>
#include <map>
#include <string>

#include <boost/tokenizer.hpp>

class COREUTILS_API Std
{
public:

    //
    // String functions
    //

    static std::vector<std::string> tokenize(const std::string& string, const std::string& separator)
    {
        std::vector<std::string> tokens;
        boost::tokenizer<boost::char_separator<char>> tokenizer(string, boost::char_separator<char>(separator.c_str()));
        for (auto& token : tokenizer) {
            tokens.push_back(token);
        }
        return tokens;
    }

    //
    // Container functions
    //

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
            std::memcpy(dest.data(), src.data(), src_bytes);
        }
        return dest;
    }
};
