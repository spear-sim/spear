//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h>    // size_t
#include <stdint.h>    // uint64_t

#include <algorithm>   // std::ranges::all_of, std::ranges::any_of, std::ranges::find, std::ranges::copy, std::ranges::equal, std::ranges::sort,
                       // std::ranges::set_intersection, std::ranges::unique
#include <concepts>    // std::convertible_to, std::same_as
#include <cstdlib>     // std::strtoull
#include <cstring>     // std::memcpy
#include <initializer_list>
#include <iterator>    // std::back_inserter, std::ranges::distance
#include <map>
#include <ranges>      // std::ranges::begin, std::ranges::contiguous_range, std::ranges::data, std::ranges::end, std::ranges::range,
                       // std::ranges::sized_range, std::ranges::size, std::views::keys, std::views::transform
#include <type_traits> // std::underlying_type_t
#include <span>
#include <string>
#include <utility>     // std::forward
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"

// TODO: remove platform-specific include
#if BOOST_COMP_MSVC
    #include <format>
#endif

// These operators are needed so an enum class can be used like an integer to represent bit flags. Note that
// operator|| and operator! are needed for SP_ASSERT
#define SP_DECLARE_ENUM_FLAG_OPERATORS(TEnum) \
    static TEnum operator|(TEnum lhs, TEnum rhs) { return static_cast<TEnum>(static_cast<std::underlying_type_t<TEnum>>(lhs) | static_cast<std::underlying_type_t<TEnum>>(rhs)); }; \
    static TEnum operator&(TEnum lhs, TEnum rhs) { return static_cast<TEnum>(static_cast<std::underlying_type_t<TEnum>>(lhs) & static_cast<std::underlying_type_t<TEnum>>(rhs)); }; \
    static bool operator||(TEnum lhs, bool rhs) { return static_cast<std::underlying_type_t<TEnum>>(lhs) || rhs; };                                                                 \
    static bool operator!(TEnum val) { return !static_cast<std::underlying_type_t<TEnum>>(val); };

//
// Helper concepts
//

template <typename TDest, typename TSrc>
concept CConvertibleFrom = std::convertible_to<TSrc, TDest>;

//
// Range (e.g., std::ranges::range) concepts
//

template <typename TRange, typename TValue>
concept CRangeHasValuesConvertibleTo =
    std::ranges::range<TRange> &&
    requires(TRange range) {
        { *std::ranges::begin(range) } -> std::convertible_to<TValue>;
    };

//
// Vector (e.g., std::span, std::vector) concepts
//

template <typename TVector>
concept CVector =
    std::ranges::sized_range<TVector> &&
    std::ranges::contiguous_range<TVector> &&
    requires (TVector vector) {
        typename TVector::value_type;
        { *(vector.begin()) } -> std::convertible_to<typename TVector::value_type>;
        { vector[0] } -> std::convertible_to<typename TVector::value_type>;
        { vector.data() } -> std::convertible_to<const typename TVector::value_type*>;
        { vector.size() } -> std::same_as<size_t>;
    };

template <typename TDestVector, typename TSrcValue>
concept CVectorHasValuesConvertibleFrom =
    CVector<TDestVector> &&
    CConvertibleFrom<typename TDestVector::value_type, TSrcValue>;

template <typename TDestVector, typename TSrcVector>
concept CVectorHasValuesConvertibleFromContainer =
    CVector<TDestVector> &&
    CVector<TSrcVector> &&
    CConvertibleFrom<typename TDestVector::value_type, typename TSrcVector::value_type>;

//
// Map (e.g., std::map) concepts
//

// We use std::move(value) here, because mapped_type might be movable but not copyable, in which case std::map::insert(...)
// won't accept an std::pair<key_type, mapped_type>, but it will accept an std::pair<key_type, mapped_type&&>.
template <typename TMap>
concept CMap =
    std::ranges::range<TMap> &&
    requires(TMap map) {
        typename TMap::key_type;
        typename TMap::mapped_type;
    } &&
    requires(TMap map, typename TMap::key_type key, typename TMap::mapped_type value) {
        { (*(map.begin())).first } -> std::convertible_to<typename TMap::key_type>;
        { (*(map.begin())).second } -> std::convertible_to<const typename TMap::mapped_type&>;
        { map.at(key) } -> std::convertible_to<const typename TMap::mapped_type&>;
        { map.contains(key) } -> std::same_as<bool>;
        { map.insert({key, std::move(value)}).first };
        { map.insert({key, std::move(value)}).second } -> std::convertible_to<bool>;
        { (*(map.insert({key, std::move(value)}).first)).first } -> std::convertible_to<typename TMap::key_type>;
        { (*(map.insert({key, std::move(value)}).first)).second } -> std::convertible_to<const typename TMap::mapped_type&>;
        { map.erase(key) } -> std::same_as<size_t>;
    };

template <typename TDestMap, typename TSrcKey>
concept CMapHasKeysConvertibleFrom =
    CMap<TDestMap> &&
    CConvertibleFrom<typename TDestMap::key_type, TSrcKey>;

template <typename TDestMap, typename TSrcValue>
concept CMapHasValuesConvertibleFrom =
    CMap<TDestMap> &&
    CConvertibleFrom<typename TDestMap::mapped_type, TSrcValue>;

template <typename TDestMap, typename TSrcKeyContainer>
concept CMapHasKeysConvertibleFromVector =
    CMap<TDestMap> &&
    CConvertibleFrom<typename TDestMap::key_type, typename TSrcKeyContainer::value_type>;

template <typename TDestMap, typename TSrcInitializerListValue>
concept CMapHasValuesConvertibleFromInitializerList =
    CMap<TDestMap> &&
    requires(TDestMap dest_map, typename TDestMap::key_type key, std::initializer_list<TSrcInitializerListValue> src_initializer_list) {
        { dest_map.insert({key, src_initializer_list}).first };
        { dest_map.insert({key, src_initializer_list}).second } -> std::convertible_to<bool>;
        { (*(dest_map.insert({key, src_initializer_list}).first)).first } -> std::convertible_to<typename TDestMap::key_type>;
        { (*(dest_map.insert({key, src_initializer_list}).first)).second } -> std::convertible_to<const typename TDestMap::mapped_type&>;
    };

template <typename TDestMap>
concept CMapHasValuesConvertibleFromEmptyInitializerList =
    requires(TDestMap dest_map, typename TDestMap::key_type key) {
        { dest_map.insert({key, {}}).first };
        { dest_map.insert({key, {}}).second } -> std::convertible_to<bool>;
        { (*(dest_map.insert({key, {}}).first)).first } -> std::convertible_to<typename TDestMap::key_type>;
        { (*(dest_map.insert({key, {}}).first)).second } -> std::convertible_to<const typename TDestMap::mapped_type&>;
    };

template <typename TDestMap, typename TSrcMap>
concept CMapHasKeysAndValuesConvertibleFromMap =
    CMap<TDestMap> &&
    CMap<TSrcMap> &&
    requires(TDestMap dest_map, TSrcMap src_map) {
        { dest_map.insert(src_map.begin(), src_map.end()) } -> std::same_as<void>;
    };

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

    static std::string toString(const auto&... args)
    {
        return (... + boost::lexical_cast<std::string>(args));
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

    template <typename TRange> requires
        CRangeHasValuesConvertibleTo<TRange, std::string>
    static std::string join(TRange&& range, const std::string& delim)
    {
        return boost::algorithm::join(std::forward<decltype(range)>(range), delim);
    }

    //
    // std::ranges::range functions
    //

    // TODO: replace with std::ranges::to<std::vector> in C++23
    template <typename TValue, typename TRange> requires
        CRangeHasValuesConvertibleTo<TRange, TValue>
    static std::vector<TValue> toVector(TRange&& range)
    {
        std::vector<TValue> vector;
        std::ranges::copy(std::forward<decltype(range)>(range), std::back_inserter(vector));
        return vector;
    }

    // TODO: replace with std::ranges::to<std::map> in C++23
    template <typename TKey, typename TValue, typename TRange> requires
        CRangeHasValuesConvertibleTo<TRange, std::pair<TKey, TValue>>
    static std::map<TKey, TValue> toMap(TRange&& range)
    {
        std::vector<std::pair<TKey, TValue>> pairs = toVector<std::pair<TKey, TValue>>(std::forward<decltype(range)>(range));
        std::vector<TKey> keys = toVector<TKey>(pairs | std::views::transform([](const auto& pair) { const auto& [key, value] = pair; return key; }));
        SP_ASSERT(allUnique(keys));
        return std::map<TKey, TValue>(pairs.begin(), pairs.end());
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
    // Vector (e.g., std::span, std::vector) functions
    //

    template <typename TVector> requires
        CVector<TVector>
    static auto toSpan(TVector& vector)
    {
        using TValue = typename TVector::value_type;
        
        return Std::reinterpretAsSpanOf<TValue>(vector);
    }

    template <typename TVector> requires
        CVector<TVector>
    static auto toSpan(const TVector& vector)
    {
        using TValue = typename TVector::value_type;

        return Std::reinterpretAsSpanOf<const TValue>(vector);
    }

    template <typename TVector, typename TValue> requires
        CVectorHasValuesConvertibleFrom<TVector, TValue>
    static bool contains(const TVector& vector, const TValue& value)
    {
        return std::ranges::find(vector, value) != vector.end();
    }

    template <typename TVector, typename TValues> requires
        CVectorHasValuesConvertibleFromContainer<TVector, TValues>
    static std::vector<bool> contains(const TVector& vector, const TValues& values)
    {
        return toVector<bool>(values | std::views::transform([&vector](const auto& value) { return contains(vector, value); }));
    }

    template <typename TVector> requires
        CVector<TVector>
    static bool isSubsetOf(const TVector& small, const TVector& big)
    {
        return all(contains(big, small));
    }

    template <typename TVector> requires
        CVector<TVector>
    static auto at(const TVector& vector, int index) // TODO:: replace with vector.at() in C++26
    {
        SP_ASSERT(index < vector.size());
        return vector[index];
    }

    template <typename TVector, typename TValue> requires
        CVectorHasValuesConvertibleFrom<TVector, TValue>
    static int index(const TVector& vector, const TValue& value)
    {
        int index = std::ranges::distance(vector.begin(), std::ranges::find(vector, value));
        return (index < vector.size()) ? index : -1;
    }

    template <typename TVector> requires
        CVector<TVector>
    static bool allUnique(const TVector& vector)
    {
        return vector.size() == unique(vector).size();
    }

    template <typename TVector> requires
        CVector<TVector>
    static auto unique(const TVector& vector)
    {
        using TValue = typename TVector::value_type;

        std::vector<TValue> values = toVector<TValue>(vector);
        std::ranges::sort(values);
        auto [begin, end] = std::ranges::unique(values);
        values.erase(begin, end);
        return values;
    }

    template <typename TVectorKeys, typename TVectorValues> requires
        CVector<TVectorKeys> && CVector<TVectorValues>
    static auto zip(const TVectorKeys& keys, const TVectorValues& values)
    {
        using TKey = typename TVectorKeys::value_type;
        using TValue = typename TVectorValues::value_type;

        SP_ASSERT(keys.size() == values.size());
        int num_elements = keys.size();
        std::map<TKey, TValue> map;
        for (int i = 0; i < num_elements; i++) {
            insert(map, at(keys, i), at(values, i));
        }
        return map;
    }

    //
    // Map (e.g., std::map) functions
    //

    template <typename TMap, typename TKey> requires
        CMapHasKeysConvertibleFrom<TMap, TKey>
    static bool containsKey(const TMap& map, const TKey& key)
    {
        return map.contains(key);
    }

    template <typename TMap, typename TVectorKeys> requires
        CMapHasKeysConvertibleFromVector<TMap, TVectorKeys> &&
        CVector<TVectorKeys>
    static std::vector<bool> containsKeys(const TMap& map, const TVectorKeys& keys)
    {
        return contains(Std::keys(map), keys); // need fully qualified Std::keys(...) because keys is a local variable
    }

    template <typename TMap, typename TVectorKeys> requires
        CMapHasKeysConvertibleFromVector<TMap, TVectorKeys>
    static auto at(const TMap& map, const TVectorKeys& keys)
    {
        using TValue = typename TMap::mapped_type;

        // if no default value is provided, then all keys must be present
        SP_ASSERT(all(containsKeys(map, keys)));
        return toVector<TValue>(keys | std::views::transform([&map](const auto& key) { return map.at(key); }));
    }

    template <typename TMap, typename TVectorKeys, typename TValue> requires
        CMapHasKeysConvertibleFromVector<TMap, TVectorKeys> &&
        CMapHasValuesConvertibleFrom<TMap, TValue>
    static auto at(const TMap& map, const TVectorKeys& keys, const TValue& default_value)
    {
        // not necessarily the same as TValue, e.g., if we pass in nullptr
        using TMapValue = typename TMap::mapped_type;

        // since a default value is provided, we don't require all keys to be present
        return toVector<TMapValue>(keys | std::views::transform([&map, &default_value](const auto& key) { return containsKey(map, key) ? map.at(key) : default_value; }));
    }

    // We use TValue&& because we want to preserve and forward the const-ness and rvalue-ness of value.
    template <typename TMap, typename TKey, typename TValue> requires
        CMapHasKeysConvertibleFrom<TMap, TKey> &&
        CMapHasValuesConvertibleFrom<TMap, TValue>
    static void insert(TMap& map, const TKey& key, TValue&& value)
    {
        auto [itr, success] = map.insert({key, std::forward<decltype(value)>(value)});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }

    // An std::initializer_list type (e.g., std::initializer_list<int>) can't be inferred as a template parameter.
    // Only the value type of the initializer list can be inferred (e.g., the int type in std::initializer_list<int>),
    // and only when the initializer list passed to the templated function is non-empty. So this insert(...)
    // specialization is required handle situations where a caller passes in a non-empty initializer list.
    template <typename TMap, typename TKey, typename TInitializerListValue> requires
        CMapHasKeysConvertibleFrom<TMap, TKey> &&
        CMapHasValuesConvertibleFromInitializerList<TMap, TInitializerListValue>
    static void insert(TMap& map, const TKey& key, std::initializer_list<TInitializerListValue> initializer_list)
    {
        auto [itr, success] = map.insert({key, initializer_list});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }

    // This insert(...) specialization is required to handle situations where a caller passes in an empty initializer
    // list. Perhaps surprisingly, this specialization behaves as expected even though EmptyInitializerList is a
    // private class.
private:
    class EmptyInitializerList;
public:
    template <typename TMap, typename TKey> requires
        CMapHasKeysConvertibleFrom<TMap, TKey> &&
        CMapHasValuesConvertibleFromEmptyInitializerList<TMap>
    static void insert(TMap& map, const TKey& key, std::initializer_list<EmptyInitializerList> initializer_list)
    {
        auto [itr, success] = map.insert({key, {}});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }

    template <typename TDestMap, typename TSrcMap> requires
        CMapHasKeysAndValuesConvertibleFromMap<TDestMap, TSrcMap>
    static void insert(TDestMap& dest_map, const TSrcMap& src_map)
    {
        using TKey = typename TDestMap::key_type;

        // Assert if there are duplicate keys.
        std::vector<TKey> keys_set_intersection;
        std::ranges::set_intersection(keys(dest_map), keys(src_map), std::back_inserter(keys_set_intersection));
        SP_ASSERT(keys_set_intersection.empty());

        dest_map.insert(std::ranges::begin(src_map), std::ranges::end(src_map));
    }

    template <typename TMap, typename TKey> requires
        CMapHasKeysConvertibleFrom<TMap, TKey>
    static void remove(TMap& map, const TKey& key)
    {
        int num_elements_removed = map.erase(key);
        SP_ASSERT(num_elements_removed == 1);
    }

    template <typename TMap, typename TVectorKeys> requires
        CMapHasKeysConvertibleFromVector<TMap, TVectorKeys> &&
        CVector<TVectorKeys>
    static void remove(TMap& map, const TVectorKeys& keys)
    {
        for (auto& key : keys) {
            remove(map, key);
        }
    }

    template <typename TMap> requires
        CMap<TMap>
    static auto keys(const TMap& map)
    {
        using TKey = typename TMap::key_type;

        // TODO: remove platform-specific logic in C++23
        #if BOOST_COMP_MSVC
            return toVector<TKey>(std::views::keys(map));
        #elif BOOST_COMP_CLANG
            std::vector<TKey> keys;
            boost::copy(map | boost::adaptors::map_keys, std::back_inserter(keys));
            return keys;
        #else
            #error
        #endif
    }

    template <typename TMap> requires
        CMap<TMap>
    static auto zip(const TMap& map)
    {
        using TKey = typename TMap::key_type;
        using TValue = typename TMap::mapped_type;

        std::vector<TKey> keys;
        std::vector<TValue> values;
        for (const auto& [key, value] : map) {
            keys.push_back(key);
            values.push_back(value);
        }
        return std::make_pair(std::move(keys), std::move(values));
    }

    //
    // Functions for safely reinterpreting ranges, spans, and initializer lists
    //

    // We use TSrcVector&& because want to preserve and forward the const-ness and rvalue-ness of src. We do
    // this to enforce the constraint that if TSrcVector is const, then TDestValue also needs to be const.
    template <typename TDestValue, typename TSrcVector> requires
        CVector<std::remove_const_t<std::remove_reference_t<TSrcVector>>> &&
        (!std::is_const_v<TSrcVector> || std::is_const_v<TDestValue>)
    static std::span<TDestValue> reinterpretAsSpanOf(TSrcVector&& src)
    {
        return reinterpretAsSpan<TDestValue>(src.data(), src.size());
    }

    // If TSrcValue is const, then TDestValue also needs to be const.
    template <typename TDestValue, typename TSrcValue> requires
        (!std::is_const_v<TSrcValue> || std::is_const_v<TDestValue>)
    static std::span<TDestValue> reinterpretAsSpan(TSrcValue* src_data, uint64_t src_num_elements)
    {
        std::span<TDestValue> dest;
        if (src_num_elements > 0) {
            uint64_t src_num_bytes = src_num_elements * sizeof(TSrcValue);
            SP_ASSERT(src_num_bytes % sizeof(TDestValue) == 0);
            uint64_t dest_num_elements = src_num_bytes / sizeof(TDestValue);
            dest = std::span<TDestValue>(reinterpret_cast<TDestValue*>(src_data), dest_num_elements);
        }
        return dest;
    }

    template <typename TDestValue, typename TSrcVector> requires
        CVector<TSrcVector>
    static std::vector<TDestValue> reinterpretAsVectorOf(const TSrcVector& src)
    {
        return reinterpretAsVector<TDestValue>(src.data(), src.size());
    }

    // Don't infer TSrcValue from the input in the functions below, because we want to force the user to
    // explicitly specify the intended value type of the input somewhere. If we allow the value type of
    // the input to be inferred automatically, we create a situation where, e.g., the user wants to create
    // an initializer list of floats, but accidentally creates an initializer list of doubles. We don't need
    // this extra layer of safety for reinterpretAsVectorOf(...), because the input to that function is,
    // e.g., an std::vector, where the user would have already specified its value type somewhere in their
    // code.

    template <typename TDestValue, typename TSrcValue, typename TRange> requires
        CRangeHasValuesConvertibleTo<TRange, TSrcValue>
    static std::vector<TDestValue> reinterpretAsVector(TRange&& range)
    {
        SP_ASSERT(sizeof(TSrcValue) % sizeof(TDestValue) == 0);
        int num_dest_elements_per_src_element = sizeof(TSrcValue) / sizeof(TDestValue);
        std::vector<TDestValue> dest;
        for (auto range_value : range) {
            dest.resize(dest.size() + num_dest_elements_per_src_element);
            TDestValue* dest_ptr = &(dest.at(dest.size() - num_dest_elements_per_src_element)); // get ptr after resize because data might have moved
            SP_ASSERT(dest_ptr);
            TSrcValue* src_ptr = reinterpret_cast<TSrcValue*>(dest_ptr);
            *src_ptr = range_value;
        }
        return dest;
    }

    template <typename TDestValue, typename TSrcValue, typename TSrcInitializerListValue> requires
        std::same_as<TSrcValue, TSrcInitializerListValue>
    static std::vector<TDestValue> reinterpretAsVector(std::initializer_list<TSrcInitializerListValue> src)
    {
        return reinterpretAsVector<TDestValue>(std::ranges::data(src), std::ranges::size(src));
    }

    // We can infer TSrcValue here because there is no potential for ambiguity.
    template <typename TDestValue, typename TSrcValue>
    static std::vector<TDestValue> reinterpretAsVector(const TSrcValue* src_data, uint64_t src_num_elements)
    {
        std::vector<TDestValue> dest;
        if (src_num_elements > 0) {
            uint64_t src_num_bytes = src_num_elements * sizeof(TSrcValue);
            SP_ASSERT(src_num_bytes % sizeof(TDestValue) == 0);
            uint64_t dest_num_elements = src_num_bytes / sizeof(TDestValue);
            dest.resize(dest_num_elements);
            std::memcpy(dest.data(), src_data, src_num_bytes);
        }
        return dest;
    }
};
