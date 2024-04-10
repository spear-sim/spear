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
#include <format>
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
// std::span concepts
//

template <typename TSpan>
concept CSpan =
    std::ranges::sized_range<TSpan> &&
    std::ranges::contiguous_range<TSpan>;

//
// Value container (e.g., std::vector) concepts
//

template <typename TDest, typename TSrc>
concept CConvertibleFrom = std::convertible_to<TSrc, TDest>;

template <typename TValueContainer>
concept CValueContainer =
    std::ranges::sized_range<TValueContainer> &&
    requires(TValueContainer value_container) {
        typename TValueContainer::value_type;
        { value_container.at(0) } -> std::convertible_to<typename TValueContainer::value_type>;
        { *std::ranges::begin(value_container) } -> std::convertible_to<typename TValueContainer::value_type>;
        { *(value_container.erase(std::ranges::begin(value_container), std::ranges::end(value_container))) } -> std::convertible_to<typename TValueContainer::value_type>;
    };

template <typename TDestValueContainer, typename TSrcValue>
concept CValueContainerHasValuesConvertibleFrom =
    CValueContainer<TDestValueContainer> &&
    CConvertibleFrom<typename TDestValueContainer::value_type, TSrcValue>;

template <typename TDestValueContainer, typename TSrcValueContainer>
concept CValueContainerHasValuesConvertibleFromContainer =
    CValueContainer<TDestValueContainer> &&
    CValueContainer<TSrcValueContainer> &&
    CConvertibleFrom<typename TDestValueContainer::value_type, typename TSrcValueContainer::value_type>;

//
// Key-value container (e.g., std::map) concepts
//

// Ideally we would constrain the type of .second to be convertible to TKeyValueContainer::mapped_type
// in the concept below, but doing so prevents this constraint for being satisfied, even when the input
// type is std::map. So we only constrain .first to be convertible to TKeyValueContainer::key_type.
template <typename TKeyValueContainer>
concept CKeyValueContainer =
    std::ranges::range<TKeyValueContainer> &&
    requires(TKeyValueContainer key_value_container) {
        typename TKeyValueContainer::key_type;
        typename TKeyValueContainer::mapped_type;
    } &&
    requires(TKeyValueContainer key_value_container, typename TKeyValueContainer::key_type key, typename TKeyValueContainer::mapped_type&& value) {
        { (*std::ranges::begin(key_value_container)).first } -> std::convertible_to<typename TKeyValueContainer::key_type>;
        { (*std::ranges::begin(key_value_container)).second };
        { key_value_container.insert({key, std::forward<decltype(value)>(value)}).first };
        { key_value_container.insert({key, std::forward<decltype(value)>(value)}).second } -> std::convertible_to<bool>;
        { (*(key_value_container.insert({key, std::forward<decltype(value)>(value)}).first)).first } -> std::convertible_to<typename TKeyValueContainer::key_type>;
        { (*(key_value_container.insert({key, std::forward<decltype(value)>(value)}).first)).second };
        { key_value_container.erase(key) } -> std::same_as<size_t>;
    };

template <typename TDestKeyValueContainer, typename TSrcKey>
concept CKeyValueContainerHasKeysConvertibleFrom =
    CKeyValueContainer<TDestKeyValueContainer> &&
    CConvertibleFrom<typename TDestKeyValueContainer::key_type, TSrcKey>;

template <typename TDestKeyValueContainer, typename TSrcValue>
concept CKeyValueContainerHasValuesConvertibleFrom =
    CKeyValueContainer<TDestKeyValueContainer> &&
    CConvertibleFrom<typename TDestKeyValueContainer::mapped_type, TSrcValue>;

template <typename TDestKeyValueContainer, typename TSrcKeyContainer>
concept CKeyValueContainerHasKeysConvertibleFromContainer =
    CKeyValueContainer<TDestKeyValueContainer> &&
    CConvertibleFrom<typename TDestKeyValueContainer::key_type, typename TSrcKeyContainer::value_type>;

template <typename TDestKeyValueContainer, typename TSrcInitializerListValue>
concept CKeyValueContainerHasValuesConvertibleFromInitializerList =
    CKeyValueContainer<TDestKeyValueContainer> &&
    requires(TDestKeyValueContainer key_value_container, typename TDestKeyValueContainer::key_type key, std::initializer_list<TSrcInitializerListValue> initializer_list) {
        { key_value_container.insert({key, initializer_list}).first };
        { key_value_container.insert({key, initializer_list}).second } -> std::convertible_to<bool>;
        { (*(key_value_container.insert({key, initializer_list}).first)).first } -> std::convertible_to<typename TDestKeyValueContainer::key_type>;
        { (*(key_value_container.insert({key, initializer_list}).first)).second };
    };

template <typename TDestKeyValueContainer>
concept CKeyValueContainerHasValuesConvertibleFromEmptyInitializerList =
    requires(TDestKeyValueContainer key_value_container, typename TDestKeyValueContainer::key_type key) {
        { key_value_container.insert({key, {}}).first };
        { key_value_container.insert({key, {}}).second } -> std::convertible_to<bool>;
        { (*(key_value_container.insert({key, {}}).first)).first } -> std::convertible_to<typename TDestKeyValueContainer::key_type>;
        { (*(key_value_container.insert({key, {}}).first)).second };
    };

template <typename TDestKeyValueContainer, typename TSrcKeyValueContainer>
concept CKeyValueContainerHasKeysAndValuesConvertibleFromContainer =
    CKeyValueContainer<TDestKeyValueContainer> &&
    CKeyValueContainer<TSrcKeyValueContainer> &&
    requires(TDestKeyValueContainer dest_key_value_container, TSrcKeyValueContainer src_key_value_container) {
        { dest_key_value_container.insert(std::ranges::begin(src_key_value_container), std::ranges::end(src_key_value_container)) } -> std::same_as<void>;
    };

//
// Concepts for safely reinterpreting contiguous value containers (e.g., std::initializer_list and std::vector)
//

template <typename TContiguousValueContainer>
concept CContiguousValueContainer =
    CValueContainer<TContiguousValueContainer> &&
    std::ranges::contiguous_range<TContiguousValueContainer>;

class SPCORE_API Std
{
public:
    Std() = delete;
    ~Std() = delete;

    //
    // std::string functions
    //

    static bool contains(const std::string& string, const std::string& substring);
    static std::vector<std::string> tokenize(const std::string& string, const std::string& separators);
    static std::string toLower(const std::string& string);

    static std::string toString(auto&&... args)
    {
        return (... + boost::lexical_cast<std::string>(std::forward<decltype(args)>(args)));
    }

    static std::string toString()
    {
        return "";
    }

    static std::string toStringFromPtr(void* ptr)
    {
        // TODO: remove platform-specific logic
        #if BOOST_COMP_MSVC
            return std::format("{:#018x}", reinterpret_cast<uint64_t>(ptr));
        #elif BOOST_COMP_CLANG
            return (boost::format("0x%016x")%reinterpret_cast<uint64_t>(ptr)).str();
        #else
            #error
        #endif
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
    static const TValue& at(const std::span<TValue>& span, int index) // TODO:: replace with span.at() in C++26
    {
        SP_ASSERT(index < span.size());
        return span[index];
    }

    //
    // std::ranges::range functions
    //

    template <typename TValue, typename TRange> requires
        CRangeHasValuesConvertibleTo<TRange, TValue>
    static std::vector<TValue> toVector(TRange&& range) // TODO: replace with std::ranges::to<std::vector> in C++23
    {
        std::vector<TValue> vector;
        std::ranges::copy(std::forward<decltype(range)>(range), std::back_inserter(vector));
        return vector;
    }

    template <typename TKey, typename TValue, typename TRange> requires
        CRangeHasValuesConvertibleTo<TRange, std::pair<TKey, TValue>>
    static std::map<TKey, TValue> toMap(TRange&& range) // TODO: replace with std::ranges::to<std::map> in C++23
    {
        std::vector<std::pair<TKey, TValue>> pairs = toVector<std::pair<TKey, TValue>>(std::forward<decltype(range)>(range));

        // Assert if there are duplicate keys.
        std::vector<TKey> keys = toVector<TKey>(pairs | std::views::transform([](const auto& pair) { const auto& [key, value] = pair; return key; }));
        SP_ASSERT(Std::allUnique(keys));

        return std::map<TKey, TValue>(std::ranges::begin(pairs), std::ranges::end(pairs));
    }

    template <typename TRange> requires
        CRangeHasValuesConvertibleTo<TRange, bool>
    static bool all(TRange&& range)
    {
        return std::ranges::all_of(std::forward<decltype(range)>(range), [](auto val) { return val; });
    }

    template <typename TRange> requires
        CRangeHasValuesConvertibleTo<TRange, bool>
    static bool any(TRange&& range)
    {
        return std::ranges::any_of(std::forward<decltype(range)>(range), [](auto val) { return val; });
    }

    //
    // Value container (e.g., std::vector) functions
    //

    template <typename TValueContainer, typename TValue> requires
        CValueContainerHasValuesConvertibleFrom<TValueContainer, TValue>
    static bool contains(const TValueContainer& value_container, const TValue& value)
    {
        return std::ranges::find(value_container, value) != value_container.end();
    }

    template <typename TValueContainer, typename TValues> requires
        CValueContainerHasValuesConvertibleFromContainer<TValueContainer, TValues>
    static std::vector<bool> contains(const TValueContainer& value_container, const TValues& values)
    {
        return toVector<bool>(values | std::views::transform([&value_container](const auto& value) { return Std::contains(value_container, value); }));
    }

    template <typename TValueContainer>
    static auto at(const TValueContainer& value_container, int index) // TODO:: remove when we can use span.at() in C++26
    {
        return value_container.at(index);
    }

    template <typename TValueContainer, typename TValue> requires
        CValueContainerHasValuesConvertibleFrom<TValueContainer, TValue>
    static int index(const TValueContainer& value_container, const TValue& value)
    {
        int index = std::ranges::distance(std::ranges::begin(value_container), std::ranges::find(value_container, value));
        return (index < std::ranges::size(value_container)) ? index : -1;
    }

    template <typename TValueContainer> requires
        CValueContainer<TValueContainer>
    static bool allUnique(const TValueContainer& value_container)
    {
        return value_container.size() == unique(value_container).size();
    }

    template <typename TValueContainer> requires
        CValueContainer<TValueContainer>
    static auto unique(const TValueContainer& value_container)
    {
        using TValue = typename TValueContainer::value_type;

        std::vector<TValue> value_container_unique = toVector<TValue>(value_container);
        std::ranges::sort(value_container_unique);
        auto [begin, end] = std::ranges::unique(std::ranges::begin(value_container_unique), std::ranges::end(value_container_unique));
        value_container_unique.erase(begin, end);
        return value_container_unique;
    }

    template <typename TValueContainerKeys, typename TValueContainerValues> requires
        CValueContainer<TValueContainerKeys> &&
        CValueContainer<TValueContainerValues>
    static auto zip(const TValueContainerKeys& keys, const TValueContainerValues& values)
    {
        using TKey = typename TValueContainerKeys::value_type;
        using TValue = typename TValueContainerValues::value_type;

        SP_ASSERT(keys.size() == values.size());
        int num_elements = keys.size();
        std::map<TKey, TValue> map;
        for (int i = 0; i < num_elements; i++) {
            Std::insert(map, Std::at(keys, i), Std::at(values, i));
        }
        return map;
    }

    //
    // Key-value container (e.g., std::map) functions
    //

    template <typename TKeyValueContainer, typename TKey> requires
        CKeyValueContainerHasKeysConvertibleFrom<TKeyValueContainer, TKey>
    static bool containsKey(const TKeyValueContainer& key_value_container, const TKey& key)
    {
        return key_value_container.contains(key);
    }

    template <typename TKeyValueContainer, typename TKey> requires
        CKeyValueContainerHasKeysConvertibleFrom<TKeyValueContainer, TKey>
    static std::vector<bool> containsKeys(const TKeyValueContainer& key_value_container, const std::vector<TKey>& keys)
    {
        return Std::contains(Std::keys(key_value_container), keys);
    }

    template <typename TKeyValueContainer, typename TKeyContainer> requires
        CKeyValueContainerHasKeysConvertibleFromContainer<TKeyValueContainer, TKeyContainer>
    static auto at(const TKeyValueContainer& key_value_container, const TKeyContainer& keys)
    {
        using TValue = typename TKeyValueContainer::mapped_type;

        // If no default value is provided, then all keys must be present
        SP_ASSERT(Std::all(containsKeys(key_value_container, keys)));
        return toVector<TValue>(keys | std::views::transform([&key_value_container](const auto& key) { return key_value_container.at(key); }));
    }

    template <typename TKeyValueContainer, typename TKeyContainer, typename TValue> requires
        CKeyValueContainerHasKeysConvertibleFromContainer<TKeyValueContainer, TKeyContainer> &&
        CKeyValueContainerHasValuesConvertibleFrom<TKeyValueContainer, TValue>
    static auto at(const TKeyValueContainer& key_value_container, const TKeyContainer& keys, const TValue& default_value)
    {
        // not necessarily the same as TValue, e.g., if we pass in nullptr
        using TContainerReturnValue = typename TKeyValueContainer::mapped_type;

        // Since a default value is provided, we don't require all keys to be present
        return toVector<TContainerReturnValue>(
            keys |
            std::views::transform([&key_value_container, &default_value](const auto& key) {
                return containsKey(key_value_container, key) ? key_value_container.at(key) : default_value; }));
    }

    template <typename TKeyValueContainer, typename TKey, typename TValue> requires
        CKeyValueContainerHasKeysConvertibleFrom<TKeyValueContainer, TKey> &&
        CKeyValueContainerHasValuesConvertibleFrom<TKeyValueContainer, TValue>
    static void insert(TKeyValueContainer& key_value_container, const TKey& key, TValue&& value)
    {
        auto [itr, success] = key_value_container.insert({key, std::forward<decltype(value)>(value)});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }

    // An std::initializer_list type (e.g., std::initializer_list<int>) can't be inferred as a template parameter.
    // Only the value type of the initializer list can be inferred (e.g., the int type in std::initializer_list<int>),
    // and only when the initializer list passed to the templated function is non-empty. So this insert(...)
    // specialization is required handle situations where a caller passes in a non-empty initializer list.
    template <typename TKeyValueContainer, typename TKey, typename TInitializerListValue> requires
        CKeyValueContainerHasKeysConvertibleFrom<TKeyValueContainer, TKey> &&
        CKeyValueContainerHasValuesConvertibleFromInitializerList<TKeyValueContainer, TInitializerListValue>
    static void insert(TKeyValueContainer& key_value_container, const TKey& key, std::initializer_list<TInitializerListValue> initializer_list)
    {
        auto [itr, success] = key_value_container.insert({key, initializer_list});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }

    // This insert(...) specialization is required to handle situations where a caller passes in an empty initializer
    // list. Perhaps surprisingly, this specialization behaves as expected even though EmptyInitializerList is a
    // private class.
private:
    class EmptyInitializerList;
public:
    template <typename TKeyValueContainer, typename TKey> requires
        CKeyValueContainerHasKeysConvertibleFrom<TKeyValueContainer, TKey> &&
        CKeyValueContainerHasValuesConvertibleFromEmptyInitializerList<TKeyValueContainer>
    static void insert(TKeyValueContainer& key_value_container, const TKey& key, std::initializer_list<EmptyInitializerList> initializer_list)
    {
        auto [itr, success] = key_value_container.insert({key, {}});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }

    template <typename TDestKeyValueContainer, typename TSrcKeyValueContainer> requires
        CKeyValueContainerHasKeysAndValuesConvertibleFromContainer<TDestKeyValueContainer, TSrcKeyValueContainer>
    static void insert(TDestKeyValueContainer& dest_key_value_container, const TSrcKeyValueContainer& src_key_value_container)
    {
        using TKey = typename TDestKeyValueContainer::key_type;

        // Assert if there are duplicate keys.
        std::vector<TKey> keys_set_intersection;
        std::ranges::set_intersection(keys(dest_key_value_container), keys(src_key_value_container), std::back_inserter(keys_set_intersection));
        SP_ASSERT(keys_set_intersection.empty());

        dest_key_value_container.insert(std::ranges::begin(src_key_value_container), std::ranges::end(src_key_value_container));
    }

    template <typename TKeyValueContainer, typename TKey> requires
        CKeyValueContainerHasKeysConvertibleFrom<TKeyValueContainer, TKey>
    static void remove(TKeyValueContainer& key_value_container, const TKey& key)
    {
        size_t num_elements_removed = key_value_container.erase(key);
        SP_ASSERT(num_elements_removed == 1);
    }

    template <typename TKeyValueContainer, typename TValueContainer> requires
        CValueContainer<TValueContainer> &&
        CKeyValueContainerHasKeysConvertibleFrom<TKeyValueContainer, typename TValueContainer::value_type>
    static void remove(TKeyValueContainer& key_value_container, const TValueContainer& keys)
    {
        for (auto& key : keys) {
            remove(key_value_container, key);
        }
    }

    template <typename TKeyValueContainer> requires
        CKeyValueContainer<TKeyValueContainer>
    static auto keys(const TKeyValueContainer& key_value_container)
    {
        using TKey = typename TKeyValueContainer::key_type;

        // TODO: remove platform-specific logic in C++23
        #if BOOST_COMP_MSVC
            return toVector<TKey>(std::views::keys(key_value_container));
        #elif BOOST_COMP_CLANG
            std::vector<TKey> keys;
            boost::copy(key_value_container | boost::adaptors::map_keys, std::back_inserter(keys));
            return keys;
        #else
            #error
        #endif
    }

    template <typename TKeyValueContainer> requires
        CKeyValueContainer<TKeyValueContainer>
    static auto zip(const TKeyValueContainer& key_value_container)
    {
        using TKey = typename TKeyValueContainer::key_type;
        using TValue = typename TKeyValueContainer::mapped_type;

        std::vector<TKey> keys;
        std::vector<TValue> values;
        for (const auto& [key, value] : key_value_container) {
            keys.push_back(key);
            values.push_back(value);
        }
        return std::make_pair(std::move(keys), std::move(values));
    }

    //
    // Functions for safely reinterpreting contiguous value containers (e.g., std::initializer_list and std::vector)
    //

    template <typename TDestValue, typename TSrcContainer> requires CSpan<TSrcContainer> || CContiguousValueContainer<TSrcContainer>
    static std::span<TDestValue> reinterpretAsSpanOf(const TSrcContainer& src)
    {
        return reinterpretAsSpan<TDestValue>(std::ranges::data(src), std::ranges::size(src));
    }

    template <typename TDestValue, typename TSrcContainer> requires CSpan<TSrcContainer> || CContiguousValueContainer<TSrcContainer>
    static std::span<TDestValue> reinterpretAsSpanOf(TSrcContainer& src)
    {
        return reinterpretAsSpan<TDestValue>(std::ranges::data(src), std::ranges::size(src));
    }

    template <typename TDestValue, typename TSrcValue>
    static std::span<TDestValue> reinterpretAsSpan(const TSrcValue* src_data, size_t src_num_elements)
    {
        std::span<TDestValue> dest;
        if (src_num_elements > 0) {
            size_t src_num_bytes = src_num_elements * sizeof(TSrcValue);
            SP_ASSERT(src_num_bytes % sizeof(TDestValue) == 0);
            size_t dest_num_elements = src_num_bytes / sizeof(TDestValue);
            dest = std::span<TDestValue>(reinterpret_cast<TDestValue*>(src_data), dest_num_elements);
        }
        return dest;
    }

    template <typename TDestValue, typename TSrcValue>
    static std::span<TDestValue> reinterpretAsSpan(TSrcValue* src_data, size_t src_num_elements)
    {
        std::span<TDestValue> dest;
        if (src_num_elements > 0) {
            size_t src_num_bytes = src_num_elements * sizeof(TSrcValue);
            SP_ASSERT(src_num_bytes % sizeof(TDestValue) == 0);
            size_t dest_num_elements = src_num_bytes / sizeof(TDestValue);
            dest = std::span<TDestValue>(reinterpret_cast<TDestValue*>(src_data), dest_num_elements);
        }
        return dest;
    }

    template <typename TDestValue, typename TSrcContainer> requires CSpan<TSrcContainer> || CContiguousValueContainer<TSrcContainer>
    static std::vector<TDestValue> reinterpretAsVectorOf(const TSrcContainer& src)
    {
        return reinterpretAsVector<TDestValue>(std::ranges::data(src), std::ranges::size(src));
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
        return reinterpretAsVector<TDestValue>(std::ranges::data(src), std::ranges::size(src));
    }

    template <typename TDestValue, typename TSrcValue>
    static std::vector<TDestValue> reinterpretAsVector(const TSrcValue* src_data, size_t src_num_elements)
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
