//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t

#include <algorithm> // std::ranges::find, std::ranges::copy, std::ranges::equal, std::ranges::sort, std::ranges::set_intersection
#include <cstdlib>   // std::strtoull
#include <cstring>   // std::memcpy
#include <format>
#include <initializer_list>
#include <iterator>  // std::back_inserter, std::distance
#include <map>
#include <ranges>    // std::ranges::range, std::views::transform
#include <span>
#include <string>
#include <utility>   // std::forward
#include <vector>


#include <boost/algorithm/string/case_conv.hpp> // boost::algorithm::to_lower_copy
#include <boost/range/adaptor/map.hpp>          // boost::adaptors::map_keys
#include <boost/range/algorithm/copy.hpp>       // boost::copy
#include <boost/tokenizer.hpp>                  // boost::char_separator

#include "SpCore/Assert.h"
#include "SpCore/BoostLexicalCast.h"

template <typename TValueContainer>
concept CValueContainer =
    std::ranges::range<TValueContainer> &&
    requires {
        typename TValueContainer::value_type;
    };

template <typename TKeyValueContainer>
concept CKeyValueContainer =
    std::ranges::range<TKeyValueContainer> &&
    requires {
        typename TKeyValueContainer::key_type;
        typename TKeyValueContainer::mapped_type;
    };

template <typename TValueContainer>
concept CContiguousValueContainer =
    CValueContainer<TValueContainer> &&
    requires(TValueContainer contiguous_value_container) {
        { contiguous_value_container.data() } -> std::same_as<typename TValueContainer::value_type*>;
        { contiguous_value_container.size() } -> std::same_as<typename TValueContainer::size_type>;
    };

class SPCORE_API Std
{
public:
    Std() = delete;
    ~Std() = delete;

    //
    // std::string functions
    //

    static bool contains(const std::string& string, const std::string& substring)
    {
        return string.find(substring) != std::string::npos;
    }

    static std::vector<std::string> tokenize(const std::string& string, const std::string& separators)
    {
        boost::tokenizer<boost::char_separator<char>> tokenizer(string, boost::char_separator<char>(separators.c_str()));
        return std::vector<std::string>(tokenizer.begin(), tokenizer.end());
    }

    static std::string toLower(const std::string& string)
    {
        return boost::algorithm::to_lower_copy(string);
    }

    static std::string toString(auto&&... args)
    {
        return (... + boost::lexical_cast<std::string>(std::forward<decltype(args)>(args)));
    }

    static std::string toStringFromPtr(void* ptr)
    {
        return std::format("{:#018x}", reinterpret_cast<uint64_t>(ptr));
    }

    template <typename TPtr>
    static TPtr* toPtrFromString(const std::string& string)
    {
        return reinterpret_cast<TPtr*>(std::strtoull(string.c_str(), nullptr, 16));
    }

    //
    // std::span functions
    //

    template <typename TData>
    static const TData& at(const std::span<TData>& span, int index)
    {
        SP_ASSERT(index < span.size());
        return span[index];
    }

    //
    // std::ranges::range functions
    //

    template <typename TData>
    static std::vector<TData> toVector(std::ranges::range auto&& range)
    {
        std::vector<TData> vector;
        std::ranges::copy(std::forward<decltype(range)>(range), std::back_inserter(vector));
        return vector;
    }

    template <typename TKey, typename TValue>
    static std::map<TKey, TValue> toMap(std::ranges::range auto&& range)
    {
        auto pairs = toVector<std::pair<TKey, TValue>>(std::forward<decltype(range)>(range));

        // Assert if there are duplicate keys.
        auto keys = toVector<TKey>(pairs | std::views::transform([](const auto& pair) { const auto& [key, value] = pair; return key; }));
        std::ranges::sort(keys);
        SP_ASSERT(std::ranges::equal(keys, unique(keys)));

        return std::map<TKey, TValue>(pairs.begin(), pairs.end());
    }

    //
    // Value container (e.g., std::vector) functions
    //

    template <CValueContainer TValueContainer>
    static bool contains(const TValueContainer& value_container, const typename TValueContainer::value_type& value)
    {
        return std::ranges::find(value_container, value) != value_container.end();
    }

    template <CValueContainer TValueContainer>
    static std::vector<bool> contains(const TValueContainer& value_container, const TValueContainer& values)
    {
        return Std::toVector<bool>(values | std::views::transform([&value_container](const auto& value) { return Std::contains(value_container, value); }));
    }

    template <CValueContainer TValueContainer>
    static int index(const TValueContainer& value_container, const typename TValueContainer::value_type& value)
    {
        int index = std::distance(value_container.begin(), std::ranges::find(value_container, value));
        return (index < value_container.size()) ? index : -1;
    }

    template <CValueContainer TValueContainer>
    static std::vector<typename TValueContainer::value_type> unique(const TValueContainer& value_container)
    {
        auto value_container_sorted_unique = value_container;
        std::ranges::sort(value_container_sorted_unique);
        const auto [begin, end] = std::ranges::unique(value_container_sorted_unique.begin(), value_container_sorted_unique.end());
        value_container_sorted_unique.erase(begin, end);
        return Std::toVector<typename TValueContainer::value_type>(value_container_sorted_unique);
    }

    template <CValueContainer TValueContainer> requires std::convertible_to<typename TValueContainer::value_type, bool>
    static bool all(const TValueContainer& value_container)
    {
        return std::ranges::all_of(value_container, [](auto val) { return val; });
    }

    template <CValueContainer TValueContainer> requires std::convertible_to<typename TValueContainer::value_type, bool>
    static bool any(const TValueContainer& value_container)
    {
        return std::ranges::any_of(value_container, [](auto val) { return val; });
    }

    //
    // Key-value container (e.g., std::map) functions
    //

    template <CKeyValueContainer TKeyValueContainer>
    static bool containsKey(const TKeyValueContainer& key_value_container, const typename TKeyValueContainer::key_type& key)
    {
        return key_value_container.contains(key);
    }

    template <CKeyValueContainer TKeyValueContainer>
    static std::vector<typename TKeyValueContainer::key_type> keys(const TKeyValueContainer& key_value_container)
    {
        // TODO: revert back to using std::views after migrating to UE 5.3:
        //     auto view = std::views::keys(key_value_container);
        //     return std::vector<typename TKeyValueContainer::key_type>(view.begin(), view.end());
        std::vector<typename TKeyValueContainer::key_type> keys;
        boost::copy(key_value_container | boost::adaptors::map_keys, std::back_inserter(keys));
        return keys;
    }


    template <CKeyValueContainer TKeyValueContainer>
    static void insert(TKeyValueContainer& key_value_container, const typename TKeyValueContainer::key_type& key, const typename TKeyValueContainer::mapped_type& value)
    {
        auto [itr, success] = key_value_container.insert({key, value});
        SP_ASSERT(success);
    }

    template <CKeyValueContainer TKeyValueContainer>
    static void insert(TKeyValueContainer& key_value_container, const typename TKeyValueContainer::key_type& key, typename TKeyValueContainer::mapped_type&& value)
    {
        auto [itr, success] = key_value_container.insert({key, std::forward<typename TKeyValueContainer::mapped_type>(value)});
        SP_ASSERT(success);
    }

    template <CKeyValueContainer TKeyValueContainer>
    static void insert(TKeyValueContainer& dest, const TKeyValueContainer& src)
    {
        std::vector<typename TKeyValueContainer::key_type> keys_intersection;
        // TODO: revert back to using std::views after migrating to UE 5.3:
        //     std::ranges::set_intersection(std::views::keys(dest), std::views::keys(src), std::back_inserter(keys_intersection));
        std::ranges::set_intersection(keys(dest), keys(src), std::back_inserter(keys_intersection));
        SP_ASSERT(keys_intersection.empty());
        dest.insert(src.begin(), src.end());
    }

    //
    // Functions for safely reinterpreting contiguous value containers (e.g., std::initializer_list and std::vector)
    //

    template <typename TDest, CContiguousValueContainer TContiguousValueContainer>
    static std::span<TDest> reinterpretAsSpanOf(const TContiguousValueContainer& src)
    {
        std::span<TDest> dest;
        size_t src_num_elements = src.size();
        if (src_num_elements > 0) {
            size_t src_num_bytes = src_num_elements * sizeof(typename TContiguousValueContainer::value_type);
            SP_ASSERT(src_num_bytes % sizeof(TDest) == 0);
            size_t dest_num_elements = src_num_bytes / sizeof(TDest);
            dest = std::span<TDest>(reinterpret_cast<TDest*>(src.data()), dest_num_elements);
        }
        return dest;
    }

    template <typename TDest, CContiguousValueContainer TContiguousValueContainer>
    static std::vector<TDest> reinterpretAsVectorOf(const TContiguousValueContainer& src)
    {
        return reinterpretAsVectorImpl<TDest, typename TContiguousValueContainer::value_type>(src.data(), src.size());
    }

    // Don't infer TSrc from the input initializer list, because we want to force the user to explicitly specify the
    // intended data type of the initializer list somewhere. If we allow TSrc to be inferred automatically, we create
    // a situation where, e.g., the user wants to create an initializer list of floats, but accidentally creates an
    // initializer list of doubles. We don't need this extra layer of safety for reinterpretAsVectorOf(...), because
    // the input to that function is, e.g., an std::vector, where the user would have already specified its data type
    // somewhere in their code.
    template <typename TDest, typename TSrc, typename TInitializerList> requires std::same_as<TSrc, TInitializerList>
    static std::vector<TDest> reinterpretAsVector(const std::initializer_list<TInitializerList>& src)
    {
        return reinterpretAsVectorImpl<TDest, TSrc>(std::data(src), src.size());
    }

    template <typename TDest, typename TSrc>
    static std::vector<TDest> reinterpretAsVectorImpl(const TSrc* src_data, size_t src_num_elements)
    {
        std::vector<TDest> dest;
        if (src_num_elements > 0) {
            size_t src_num_bytes = src_num_elements * sizeof(TSrc);
            SP_ASSERT(src_num_bytes % sizeof(TDest) == 0);
            size_t dest_num_elements = src_num_bytes / sizeof(TDest);
            dest.resize(dest_num_elements);
            std::memcpy(dest.data(), src_data, src_num_bytes);
        }
        return dest;
    }
};
