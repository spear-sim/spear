//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t

#include <algorithm> // std::find
#include <cctype>    // std::tolower
#include <cstring>   // std::memcpy
#include <iterator>  // std::distance
#include <string>
#include <utility>   // std::forward
#include <vector>

#include <boost/tokenizer.hpp> // boost::char_separator

#include "CoreUtils/Assert.h"
#include "CoreUtils/BoostLexicalCast.h"

class COREUTILS_API Std
{
public:
    Std() = delete;
    ~Std() = delete;

    //
    // String conversion functions
    //

    static std::string toString()
    {
        return "";
    }

    template <typename TArg>
    static std::string toString(const TArg& arg)
    {
        return boost::lexical_cast<std::string>(arg);
    }

    template <typename TArg, class... TArgs>
    static std::string toString(const TArg& front, TArgs&&... back)
    {
        return toString(front) + toString(std::forward<TArgs>(back)...);
    }

    static std::vector<std::string> tokenize(const std::string& string, const std::string& separators)
    {
        std::vector<std::string> tokens;
        boost::tokenizer<boost::char_separator<char>> tokenizer(string, boost::char_separator<char>(separators.c_str()));
        for (auto& token : tokenizer) {
            tokens.push_back(token);
        }
        return tokens;
    }

    static bool containsSubstring(const std::string& string, const std::string& substring)
    {
        return string.find(substring) != std::string::npos;
    }

    static std::string toLower(const std::string& string) {
        std::string lower_string = string;
        for (auto& c : lower_string) {
            c = std::tolower(c);
        }
        return lower_string;
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

    template <typename TContainer, typename TKey>
    static int index(const TContainer& container, const TKey& key)
    {
        int index = std::distance(container.begin(), std::find(container.begin(), container.end(), key));

        if (index < container.size()) {
            return index;
        } else {
            return -1;
        }
    }

    //
    // Reinterpet a vector of a given type as a vector of a different type
    //

    template <typename TDest, typename TSrc>
    static std::vector<TDest> reinterpretAs(const std::vector<TSrc>& src)
    {
        std::vector<TDest> dest;
        if (src.size() > 0) {
            size_t src_bytes = src.size() * sizeof(TSrc);
            SP_ASSERT(src_bytes % sizeof(TDest) == 0);
            size_t dest_elements = src_bytes / sizeof(TDest);
            dest.resize(dest_elements);
            std::memcpy(dest.data(), src.data(), src_bytes);
        }
        return dest;
    }
};
