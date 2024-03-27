//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // uint64_t

#include <algorithm> // std::ranges::all_of, std::ranges::any_of, std::ranges::find, std::ranges::copy, std::ranges::equal, std::ranges::sort,
                     // std::ranges::set_intersection, std::ranges::unique
#include <concepts>  // std::convertible_to, std::same_as
#include <cstdlib>   // std::strtoull
#include <cstring>   // std::memcpy
#include <initializer_list>
#include <iterator>  // std::back_inserter, std::ranges::distance
#include <map>
#include <ranges>    // std::ranges::begin, std::ranges::contiguous_range, std::ranges::data, std::ranges::end, std::ranges::range,
                     // std::ranges::sized_range, std::ranges::size, std::views::keys, std::views::transform
#include <span>
#include <string>
#include <utility>   // std::forward
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"

//
// std::ranges::range concepts
//

template <typename TRange, typename TValue>
concept CRangeHasValuesConvertibleTo =
    std::ranges::range<TRange> &&
    requires(TRange&& range) {
        { *std::ranges::begin(range) } -> std::convertible_to<TValue>;
    };

//
// Value container (e.g., std::vector) concepts
//

template <typename TDest, typename TSrc>
concept CConvertibleFrom = std::convertible_to<TSrc, TDest>;

template <typename TValueContainer>
concept CValueContainer =
    std::ranges::range<TValueContainer> &&
    requires(TValueContainer value_container) {
        typename TValueContainer::value_type;
        { *std::ranges::begin(value_container) } -> std::convertible_to<typename TValueContainer::value_type>;
    };

template <typename TValueContainer, typename TValue>
concept CValueContainerHasValuesConvertibleFrom =
    CValueContainer<TValueContainer> &&
    CConvertibleFrom<typename TValueContainer::value_type, TValue>;

template <typename TValueContainer, typename TValues>
concept CValueContainerConvertibleFrom =
    CValueContainer<TValueContainer> &&
    CValueContainer<TValues> &&
    CConvertibleFrom<typename TValueContainer::value_type, typename TValues::value_type>;

//
// Key-value container (e.g., std::map) concepts
//

// Ideally we would constrain the type of (*std::ranges::begin(...)).second to be convertible to
// TKeyValueContainer::mapped_type, but doing so prevents this constraint for being satisfied, even when
// the input type is std::map. So we only constrain (*std::ranges::begin(...)).first to be convertible
// to TKeyValueContainer::key_type.
template <typename TKeyValueContainer>
concept CKeyValueContainer =
    std::ranges::range<TKeyValueContainer> &&
    requires(TKeyValueContainer key_value_container) {
        typename TKeyValueContainer::key_type;
        typename TKeyValueContainer::mapped_type;
        { (*std::ranges::begin(key_value_container)).first } -> std::convertible_to<typename TKeyValueContainer::key_type>;
        { (*std::ranges::begin(key_value_container)).second };
    };

template <typename TKeyValueContainer, typename TValue>
concept CKeyValueContainerHasKeysConvertibleFrom =
    CKeyValueContainer<TKeyValueContainer> &&
    CConvertibleFrom<typename TKeyValueContainer::key_type, TValue>;

template <typename TKeyValueContainer, typename TKey, typename TValue>
concept CKeyValueContainerCanInsertValue =
    CKeyValueContainer<TKeyValueContainer> &&
    requires(TKeyValueContainer& key_value_range, const TKey& key, TValue&& value) {
        { key_value_range.insert({key, std::forward<decltype(value)>(value)}).first };
        { key_value_range.insert({key, std::forward<decltype(value)>(value)}).second };
    };

template <typename TKeyValueContainer, typename TKey, typename TInitializerListValue>
concept CKeyValueContainerCanInsertInitializerList =
    CKeyValueContainer<TKeyValueContainer> &&
    requires(TKeyValueContainer& key_value_range, const TKey& key, std::initializer_list<TInitializerListValue> initializer_list) {
        { key_value_range.insert({key, initializer_list}).first };
        { key_value_range.insert({key, initializer_list}).second };
    };

template <typename TKeyValueContainer, typename TKey>
concept CKeyValueContainerCanInsertEmptyInitializerList =
    CKeyValueContainer<TKeyValueContainer> &&
    requires(TKeyValueContainer& key_value_range, const TKey& key) {
        { key_value_range.insert({key, {}}).first };
        { key_value_range.insert({key, {}}).second };
    };

template <typename TDestKeyValueContainer, typename TSrcKeyValueContainer>
concept CKeyValueContainerCanInsertContainer =
    CKeyValueContainer<TDestKeyValueContainer> &&
    CKeyValueContainer<TSrcKeyValueContainer> &&
    requires(TDestKeyValueContainer& dest_key_value_container, const TSrcKeyValueContainer& src_key_value_container) {
        { dest_key_value_container.insert(std::ranges::begin(src_key_value_container), std::ranges::end(src_key_value_container)) };
    };

//
// Concepts for safely reinterpreting contiguous value containers (e.g., std::initializer_list and std::vector)
//

template <typename TContiguousValueContainer>
concept CContiguousValueContainer =
    CValueContainer<TContiguousValueContainer> &&
    std::ranges::contiguous_range<TContiguousValueContainer> &&
    std::ranges::sized_range<TContiguousValueContainer>;

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
        return Std::toString("{:#018x}", reinterpret_cast<uint64_t>(ptr));
    }

    template <typename TPtr>
    static TPtr* toPtrFromString(const std::string& string)
    {
        return reinterpret_cast<TPtr*>(std::strtoull(string.c_str(), nullptr, 16));
    }

    //
    // std::span functions
    //

    template <typename TValue>
    static const TValue& at(const std::span<TValue>& span, int index)
    {
        SP_ASSERT(index < span.size());
        return span[index];
    }

    //
    // std::ranges::range functions
    //

    template <typename TValue, typename TRange> requires CRangeHasValuesConvertibleTo<TRange, TValue>
    static std::vector<TValue> toVector(TRange&& range)
    {
        std::vector<TValue> vector;
        std::ranges::copy(std::forward<decltype(range)>(range), std::back_inserter(vector));
        return vector;
    }

    template <typename TKey, typename TValue, typename TRange> requires CRangeHasValuesConvertibleTo<TRange, std::pair<TKey, TValue>>
    static std::map<TKey, TValue> toMap(TRange&& range)
    {
        std::vector<std::pair<TKey, TValue>> pairs = toVector<std::pair<TKey, TValue>>(std::forward<decltype(range)>(range));

        // Assert if there are duplicate keys.
        std::vector<TKey> keys = toVector<TKey>(pairs | std::views::transform([](const auto& pair) { const auto& [key, value] = pair; return key; }));
        std::ranges::sort(keys);
        SP_ASSERT(std::ranges::equal(keys, unique(keys)));

        return std::map<TKey, TValue>(std::ranges::begin(pairs), std::ranges::end(pairs));
    }

    template <typename TRange> requires CRangeHasValuesConvertibleTo<TRange, bool>
    static bool all(TRange&& range)
    {
        return std::ranges::all_of(std::forward<decltype(range)>(range), [](auto val) { return val; });
    }

    template <typename TRange> requires CRangeHasValuesConvertibleTo<TRange, bool>
    static bool any(TRange&& range)
    {
        return std::ranges::any_of(std::forward<decltype(range)>(range), [](auto val) { return val; });
    }

    //
    // Value container (e.g., std::vector) functions
    //

    template <typename TValueContainer, typename TValue> requires CValueContainerHasValuesConvertibleFrom<TValueContainer, TValue>
    static bool contains(const TValueContainer& value_container, const TValue& value)
    {
        return std::ranges::find(value_container, value) != value_container.end();
    }

    template <typename TValueContainer, typename TValues> requires CValueContainerConvertibleFrom<TValueContainer, TValues>
    static std::vector<bool> contains(const TValueContainer& value_container, const TValues& values)
    {
        return toVector<bool>(values | std::views::transform([&value_container](const auto& value) { return Std::contains(value_container, value); }));
    }

    template <typename TValueContainer, typename TValue> requires CValueContainerHasValuesConvertibleFrom<TValueContainer, TValue>
    static int index(const TValueContainer& value_container, const TValue& value)
    {
        int index = std::ranges::distance(std::ranges::begin(value_container), std::ranges::find(value_container, value));
        return (index < std::ranges::size(value_container)) ? index : -1;
    }

    template <typename TValueContainer> requires CValueContainer<TValueContainer>
    static std::vector<typename TValueContainer::value_type> unique(const TValueContainer& value_container)
    {
        using TValue = TValueContainer::value_type;

        std::vector<TValue> value_container_unique = toVector<TValue>(value_container);
        std::ranges::sort(value_container_unique);
        auto [begin, end] = std::ranges::unique(std::ranges::begin(value_container_unique), std::ranges::end(value_container_unique));
        value_container_unique.erase(begin, end);
        return value_container_unique;
    }

    //
    // Key-value container (e.g., std::map) functions
    //

    template <typename TKeyValueContainer, typename TKey> requires CKeyValueContainerHasKeysConvertibleFrom<TKeyValueContainer, TKey>
    static bool containsKey(const TKeyValueContainer& key_value_container, const TKey& key)
    {
        return key_value_container.contains(key);
    }

    template <typename TKeyValueContainer> requires CKeyValueContainer<TKeyValueContainer>
    static std::vector<typename TKeyValueContainer::key_type> keys(const TKeyValueContainer& key_value_container)
    {
        using TKey = typename TKeyValueContainer::key_type;

        // TODO: remove platform-specific logic
        #ifdef BOOST_COMP_MSVC
            return toVector<TKey>(std::views::keys(key_value_container));
        #elif BOOST_COMP_CLANG
            std::vector<TKey> keys;
            boost::copy(key_value_container | boost::adaptors::map_keys, std::back_inserter(keys));
            return keys;
        #else
            #error
        #endif
    }

    template <typename TKeyValueContainer, typename TKey, typename TValue> requires CKeyValueContainerCanInsertValue<TKeyValueContainer, TKey, TValue>
    static void insert(TKeyValueContainer& key_value_container, const TKey& key, TValue&& value)
    {
        auto [itr, success] = key_value_container.insert({key, std::forward<decltype(value)>(value)});
        SP_ASSERT(success);
    }

    // An std::initializer_list type (e.g., std::initializer_list<int>) can't be inferred as a template parameter.
    // Only the value type of the initializer list can be inferred (e.g., the int type in std::initializer_list<int>),
    // and only when the initializer list passed to the templated function is non-empty. So this insert(...)
    // specialization is required handle situations where a caller passes in a non-empty initializer list.
    template <typename TKeyValueContainer, typename TKey, typename TInitializerListValue> requires CKeyValueContainerCanInsertInitializerList<TKeyValueContainer, TKey, TInitializerListValue>
    static void insert(TKeyValueContainer& key_value_container, const TKey& key, std::initializer_list<TInitializerListValue> initializer_list)
    {
        auto [itr, success] = key_value_container.insert({key, initializer_list});
        SP_ASSERT(success);
    }

    // This insert(...) specialization is required to handle situations where a caller passes in an empty initializer
    // list. Perhaps surprisingly, this specialization behaves as expected even though EmptyInitializerList is a
    // private class.
private:
    class EmptyInitializerList;
public:
    template <typename TKeyValueContainer, typename TKey> requires CKeyValueContainerCanInsertEmptyInitializerList<TKeyValueContainer, TKey>
    static void insert(TKeyValueContainer& key_value_container, const TKey& key, std::initializer_list<EmptyInitializerList> initializer_list)
    {
        auto [itr, success] = key_value_container.insert({key, {}});
        SP_ASSERT(success);
    }

    template <typename TDestKeyValueContainer, typename TSrcKeyValueContainer> requires CKeyValueContainerCanInsertContainer<TDestKeyValueContainer, TSrcKeyValueContainer>
    static void insert(TDestKeyValueContainer& dest_key_value_container, const TSrcKeyValueContainer& src_key_value_container)
    {
        using TKey = typename TDestKeyValueContainer::key_type;

        // Assert if there are duplicate keys.
        std::vector<TKey> keys_set_intersection;
        std::ranges::set_intersection(keys(dest_key_value_container), keys(src_key_value_container), std::back_inserter(keys_set_intersection));
        SP_ASSERT(keys_set_intersection.empty());

        dest_key_value_container.insert(std::ranges::begin(src_key_value_container), std::ranges::end(src_key_value_container));
    }

    //
    // Functions for safely reinterpreting contiguous value containers (e.g., std::initializer_list and std::vector)
    //

    template <typename TDestValue, typename TSrcValueContainer> requires CContiguousValueContainer<TSrcValueContainer>
    static std::span<TDestValue> reinterpretAsSpanOf(const TSrcValueContainer& src)
    {
        using TSrcValue = typename TSrcValueContainer::value_type;

        std::span<TDestValue> dest;
        size_t src_num_elements = std::ranges::size(src);
        if (src_num_elements > 0) {
            size_t src_num_bytes = src_num_elements * sizeof(TSrcValue);
            SP_ASSERT(src_num_bytes % sizeof(TDestValue) == 0);
            size_t dest_num_elements = src_num_bytes / sizeof(TDestValue);
            dest = std::span<TDestValue>(reinterpret_cast<TDestValue*>(std::ranges::data(src)), dest_num_elements);
        }
        return dest;
    }

    template <typename TDestValue, typename TSrcValueContainer> requires CContiguousValueContainer<TSrcValueContainer>
    static std::vector<TDestValue> reinterpretAsVectorOf(const TSrcValueContainer& src)
    {
        using TSrcValue = typename TSrcValueContainer::value_type;

        return reinterpretAsVectorImpl<TDestValue, TSrcValue>(std::ranges::data(src), std::ranges::size(src));
    }

    // Don't infer TSrcValue from the input initializer list, because we want to force the user to explicitly specify
    // the intended value type of the initializer list somewhere. If we allow TSrc to be inferred automatically, we
    // create a situation where, e.g., the user wants to create an initializer list of floats, but accidentally
    // creates an initializer list of doubles. We don't need this extra layer of safety for reinterpretAsVectorOf(...),
    // because the input to that function is, e.g., an std::vector, where the user would have already specified its
    // value type somewhere in their code.
    template <typename TDestValue, typename TSrcValue, typename TSrcInitializerListValue> requires std::same_as<TSrcValue, TSrcInitializerListValue>
    static std::vector<TDestValue> reinterpretAsVector(std::initializer_list<TSrcInitializerListValue> src)
    {
        return reinterpretAsVectorImpl<TDestValue, TSrcValue>(std::ranges::data(src), std::ranges::size(src));
    }

private:
    template <typename TDestValue, typename TSrcValue>
    static std::vector<TDestValue> reinterpretAsVectorImpl(const TSrcValue* src_data, size_t src_num_elements)
    {
        std::vector<TDestValue> dest;
        if (src_num_elements > 0) {
            size_t src_num_bytes = src_num_elements * sizeof(TSrcValue);
            SP_ASSERT(src_num_bytes % sizeof(TDestValue) == 0);
            size_t dest_num_elements = src_num_bytes / sizeof(TDestValue);
            dest.resize(dest_num_elements);
            std::memcpy(std::ranges::data(dest), src_data, src_num_bytes);
        }
        return dest;
    }
};
