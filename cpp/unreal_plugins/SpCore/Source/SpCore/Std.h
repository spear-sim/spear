//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // uint64_t

#include <algorithm>   // std::ranges::all_of, std::ranges::any_of, std::ranges::find, std::ranges::copy, std::ranges::equal, std::ranges::sort,
                       // std::ranges::set_intersection, std::ranges::unique
#include <concepts>    // std::convertible_to, std::same_as
#include <cstdlib>     // std::strtoull
#include <cstring>     // std::memcpy
#include <initializer_list>
#include <iterator>    // std::back_inserter, std::ranges::distance
#include <map>
#include <memory>      // std::allocator_traits
#include <ranges>      // std::ranges::begin, std::ranges::contiguous_range, std::ranges::data, std::ranges::end, std::ranges::range,
                       // std::ranges::sized_range, std::ranges::size, std::views::keys, std::views::transform
#include <type_traits> // std::remove_cvref_t, std::underlying_type_t
#include <span>
#include <set>
#include <string>      // std::equal
#include <utility>     // std::forward, std::make_pair, std::move, std::pair
#include <vector>

#include <boost/predef.h> // BOOST_COMP_CLANG, BOOST_COMP_MSVC, BOOST_OS_LINUX

#include <HAL/Platform.h> // SPCORE_API

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"

// TODO: remove platform-specific include
#if BOOST_COMP_MSVC
    #include <format>
#endif

#if BOOST_OS_LINUX
    #ifdef __cplusplus
        #define SP_EXTERN_C extern "C"
    #else
        #define SP_EXTERN_C
    #endif
    SP_EXTERN_C SPCORE_API long int __isoc23_sscanf(const char *str, const char *format, ...);
    SP_EXTERN_C SPCORE_API long int __isoc23_strtol(const char* str, char** endptr, int base);
#endif

// These operators are needed so an enum class can be used like an integer to represent bit flags. Note that operator|| and operator! are needed for SP_ASSERT
#define SP_DECLARE_ENUM_FLAG_OPERATORS(TEnum) \
    static TEnum operator |(TEnum lhs, TEnum rhs) { return static_cast<TEnum>(static_cast<std::underlying_type_t<TEnum>>(lhs) | static_cast<std::underlying_type_t<TEnum>>(rhs)); }; \
    static TEnum operator &(TEnum lhs, TEnum rhs) { return static_cast<TEnum>(static_cast<std::underlying_type_t<TEnum>>(lhs) & static_cast<std::underlying_type_t<TEnum>>(rhs)); }; \
    static bool operator ||(TEnum lhs, bool rhs)  { return static_cast<std::underlying_type_t<TEnum>>(lhs) || rhs; };                                                                \
    static bool operator !(TEnum val)             { return !static_cast<std::underlying_type_t<TEnum>>(val); };

//
// Helper concepts
//

template <typename TDest, typename TSrc>
concept CConvertibleFrom = std::convertible_to<TSrc, TDest>;

//
// Range (e.g., std::ranges::range) concepts
//

template <typename TRange, typename TValue>
concept CRangeValuesAreConvertibleTo =
    std::ranges::range<TRange> &&
    requires (TRange range) {
        { *(std::ranges::begin(range)) } -> std::convertible_to<const TValue&>;
        { *(std::ranges::end(range)) } -> std::convertible_to<const TValue&>;
    };

//
// Vector (e.g., std::span, std::vector) concepts
//

template <typename TVector>
concept CVector =
    std::ranges::sized_range<TVector> &&
    std::ranges::contiguous_range<TVector> &&
    requires (TVector vector) {
        typename TVector::iterator;
        typename TVector::value_type;
    } &&
    requires (TVector vector, typename TVector::iterator itr) {
        { vector.begin() } -> std::convertible_to<const typename TVector::iterator&>;
        { vector.end() } -> std::convertible_to<const typename TVector::iterator&>;
        { vector[0] } -> std::convertible_to<const typename TVector::value_type&>;
        { vector.data() } -> std::convertible_to<const typename TVector::value_type*>;
        { vector.size() } -> std::same_as<size_t>;
        { *itr } -> std::convertible_to<const typename TVector::value_type&>;
    };

template <typename TDestVector, typename TSrcValue>
concept CVectorValuesAreConvertibleFrom =
    CVector<TDestVector> &&
    CConvertibleFrom<typename TDestVector::value_type, TSrcValue>;

template <typename TDestVector, typename TSrcVectorValues>
concept CVectorValuesAreConvertibleFromVector =
    CVector<TDestVector> &&
    CVector<TSrcVectorValues> &&
    CConvertibleFrom<typename TDestVector::value_type, typename TSrcVectorValues::value_type>;

//
// Map (e.g., std::map) concepts
//

template <typename TMap>
concept CMap =
    std::ranges::range<TMap> &&
    requires (TMap map) {
        typename TMap::iterator;
        typename TMap::key_type;
        typename TMap::mapped_type;
        typename TMap::value_type;
    } &&
    std::convertible_to<typename TMap::value_type, std::pair<typename TMap::key_type, typename TMap::mapped_type>> &&
    requires (TMap map, typename TMap::iterator itr, typename TMap::key_type key, typename TMap::mapped_type value) {
        { map.begin() } -> std::convertible_to<const typename TMap::iterator&>;
        { map.end() } -> std::convertible_to<const typename TMap::iterator&>;
        { map.at(key) } -> std::convertible_to<const typename TMap::mapped_type&>;
        { map.contains(key) } -> std::same_as<bool>;
        { map.insert({std::move(key), std::move(value)}) } -> std::convertible_to<const std::pair<typename TMap::iterator, bool>&>; // std::move is needed otherwise CMap<std::map> will be false
        { map.erase(key) } -> std::same_as<size_t>;
        { *itr } -> std::convertible_to<const typename TMap::value_type&>;
    };

template <typename TDestMap, typename TSrcKey>
concept CMapKeysAreConvertibleFrom =
    CMap<TDestMap> &&
    CConvertibleFrom<typename TDestMap::key_type, TSrcKey>;

template <typename TDestMap, typename TSrcVectorKeys>
concept CMapKeysAreConvertibleFromVector =
    CMap<TDestMap> &&
    CConvertibleFrom<typename TDestMap::key_type, typename TSrcVectorKeys::value_type>;

template <typename TDestMap, typename TSrcValue>
concept CMapValuesAreConvertibleFrom =
    CMap<TDestMap> &&
    CConvertibleFrom<typename TDestMap::mapped_type, TSrcValue>;

template <typename TDestMap, typename TSrcValue>
concept CMapValuesAreConvertibleFromInitializerList =
    CMap<TDestMap> &&
    requires (TDestMap dest_map, typename TDestMap::key_type key, std::initializer_list<TSrcValue> src_initializer_list) {
        { dest_map.insert({key, src_initializer_list}) } -> std::convertible_to<const std::pair<typename TDestMap::iterator, bool>&>;
    };

template <typename TDestMap>
concept CMapValuesAreConvertibleFromEmptyInitializerList =
    CMap<TDestMap> &&
    requires (TDestMap dest_map, typename TDestMap::key_type key) {
        { dest_map.insert({key, {}}) } -> std::convertible_to<const std::pair<typename TDestMap::iterator, bool>&>;
    };

template <typename TDestMap, typename TSrcMap>
concept CMapKeysAndValuesAreConvertibleFromMap =
    CMap<TDestMap> &&
    CMap<TSrcMap> &&
    requires (TDestMap dest_map, TSrcMap src_map) {
        { dest_map.insert(src_map.begin(), src_map.end()) } -> std::same_as<void>;
    };

//
// Set (e.g., std::set) concepts
//

template <typename TSet>
concept CSet =
    std::ranges::range<TSet> &&
    requires (TSet map) {
        typename TSet::iterator;
        typename TSet::key_type;
        typename TSet::value_type;
    } &&
    requires (TSet set, typename TSet::iterator itr, typename TSet::key_type key) {
        { set.begin() } -> std::convertible_to<const typename TSet::iterator&>;
        { set.end() } -> std::convertible_to<const typename TSet::iterator&>;
        { set.find(key) } -> std::convertible_to<const typename TSet::iterator&>;
        { set.contains(key) } -> std::same_as<bool>;
        { set.insert(key) } -> std::convertible_to<const std::pair<typename TSet::iterator, bool>&>;
        { set.erase(key) } -> std::same_as<size_t>;
        { *itr } -> std::convertible_to<const typename TSet::value_type&>;
    };

template <typename TDestSet, typename TSrcKey>
concept CSetKeysAreConvertibleFrom =
    CSet<TDestSet> &&
    CConvertibleFrom<typename TDestSet::key_type, TSrcKey>;

template <typename TDestSet, typename TSrcVectorKeys>
concept CSetKeysAreConvertibleFromVector =
    CSet<TDestSet> &&
    CConvertibleFrom<typename TDestSet::key_type, typename TSrcVectorKeys::value_type>;

//
// std utility functions
//

template <typename TValue, size_t TNumBytes>
using SpAlignedAllocator = boost::alignment::aligned_allocator<TValue, TNumBytes>;

class SPCORE_API Std
{
public:
    Std() = delete;
    ~Std() = delete;

    template <typename T>
    static std::string getTypeIdString()
    {
        // RTTI is not allowed in modules that define Unreal types, so we can't use typeid(T). We also can't use
        // use boost::typeindex::type_id<T>, which is intended to emulate RTTI without actually enabling it, because
        // this conflicts with some Unreal modules that explicitly enable RTTI. So we use BOOST_CURRENT_FUNCTION
        // as a lightweight alternative because it will give us a unique string for each type.
        return BOOST_CURRENT_FUNCTION;
    }

    //
    // enum class functions
    //

    template <typename TEnum> requires std::is_enum_v<TEnum>
    static bool toBool(TEnum value) {
        return static_cast<std::underlying_type_t<TEnum>>(value) != 0;
    }

    //
    // std::string functions
    //

    static bool contains(const std::string& string, const std::string& substring)
    {
        return string.find(substring) != std::string::npos;
    }

    static bool startsWith(const std::string& string, const std::string& prefix)
    {
        return std::equal(prefix.begin(), prefix.end(), string.begin());
    }

    static bool endsWith(const std::string& string, const std::string& suffix)
    {
        return std::equal(suffix.rbegin(), suffix.rend(), string.rbegin());
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

    template <typename... TArgs>
    static std::string toString(TArgs&&... args)
    {
        return (... + boost::lexical_cast<std::string>(std::forward<TArgs>(args)));
    }

    static std::string toString()
    {
        return "";
    }

    static std::string toStringFromPtr(const void* ptr)
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
        SP_ASSERT(string.starts_with("0x"));
        return reinterpret_cast<TPtr*>(std::strtoull(string.c_str(), nullptr, 16));
    }

    //
    // std::vector functions
    //

    template <typename TValue, typename... TVectorTraits>
    static void resizeUninitialized(std::vector<TValue, TVectorTraits...>& vector, uint64_t size)
    {
        // This function is unsafe because it relies on undefined behavior when invoking vector_no_default_init->resize().

        struct alignas(alignof(TValue)) TValueNoDefaultInit { uint8_t value[sizeof(TValue)]; TValueNoDefaultInit() {} };

        using TVector = std::vector<TValue, TVectorTraits...>;
        using TAllocator = typename TVector::allocator_type;
        using TVectorNoDefaultInit = std::vector<TValueNoDefaultInit, typename std::allocator_traits<TAllocator>::template rebind_alloc<TValueNoDefaultInit>>;

        SP_ASSERT(sizeof(TValue) == sizeof(TValueNoDefaultInit));
        SP_ASSERT(alignof(TValue) == alignof(TValueNoDefaultInit));

        TVectorNoDefaultInit* vector_no_default_init = reinterpret_cast<TVectorNoDefaultInit*>(&vector);
        vector_no_default_init->resize(size); // undefined behavior

        SP_ASSERT(vector.size() == size);
    }

    //
    // Range (e.g., std::ranges::range) functions
    //

    template <typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, std::string>
    static std::string join(TRange&& range, const std::string& delim)
    {
        return boost::algorithm::join(std::forward<TRange>(range), delim);
    }

    // TODO: replace with std::ranges::to<std::vector> in C++23
    template <typename TValue, typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, TValue>
    static std::vector<TValue> toVector(TRange&& range)
    {
        std::vector<TValue> vector;
        std::ranges::copy(std::forward<TRange>(range), std::back_inserter(vector));
        return vector;
    }

    // TODO: replace with std::ranges::to<std::map> in C++23
    template <typename TKey, typename TValue, typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, std::pair<TKey, TValue>>
    static std::map<TKey, TValue> toMap(TRange&& range)
    {
        std::vector<std::pair<TKey, TValue>> pairs = toVector<std::pair<TKey, TValue>>(std::forward<TRange>(range));

        // assert if there are duplicate keys
        std::vector<TKey> keys = toVector<TKey>(pairs | std::views::transform([](const auto& pair) { const auto& [key, value] = pair; return key; }));
        SP_ASSERT(allUnique(keys));

        return std::map<TKey, TValue>(pairs.begin(), pairs.end());
    }

    // TODO: replace with std::ranges::to<std::map> in C++23
    template <typename TKey, typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, TKey>
    static std::set<TKey> toSet(TRange&& range)
    {
        return std::set<TKey>(std::ranges::begin(range), std::ranges::end(range));
    }

    template <typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, bool>
    static bool all(TRange&& range)
    {
        return std::ranges::all_of(std::forward<TRange>(range), [](auto val) { return val; });
    }

    template <typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, bool>
    static bool any(TRange&& range)
    {
        return std::ranges::any_of(std::forward<TRange>(range), [](auto val) { return val; });
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
        CVectorValuesAreConvertibleFrom<TVector, TValue>
    static bool contains(const TVector& vector, const TValue& value)
    {
        return std::ranges::find(vector, value) != vector.end();
    }

    template <typename TVector, typename TValues> requires
        CVectorValuesAreConvertibleFromVector<TVector, TValues>
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
    static const auto& at(const TVector& vector, int index) // TODO:: replace with vector.at() in C++26
    {
        SP_ASSERT(index < vector.size());
        return vector[index];
    }

    template <typename TVector> requires
        CVector<TVector>
    static auto& at(TVector& vector, int index) // TODO:: replace with vector.at() in C++26
    {
        SP_ASSERT(index < vector.size());
        return vector[index];
    }

    template <typename TVector, typename TValue> requires
        CVectorValuesAreConvertibleFrom<TVector, TValue>
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
        CMapKeysAreConvertibleFrom<TMap, TKey>
    static bool containsKey(const TMap& map, const TKey& key)
    {
        return map.contains(key);
    }

    template <typename TMap, typename TVectorKeys> requires
        CMapKeysAreConvertibleFromVector<TMap, TVectorKeys> &&
        CVector<TVectorKeys>
    static std::vector<bool> containsKeys(const TMap& map, const TVectorKeys& keys)
    {
        return contains(Std::keys(map), keys); // need fully qualified Std::keys(...) because keys is a local variable
    }

    template <typename TMap, typename TVectorKeys> requires
        CMapKeysAreConvertibleFromVector<TMap, TVectorKeys>
    static auto at(const TMap& map, const TVectorKeys& keys)
    {
        using TValue = typename TMap::mapped_type;

        // if no default value is provided, then all keys must be present
        SP_ASSERT(all(containsKeys(map, keys)));
        return toVector<TValue>(keys | std::views::transform([&map](const auto& key) { return map.at(key); }));
    }

    template <typename TMap, typename TVectorKeys, typename TDefaultValue> requires
        CMapKeysAreConvertibleFromVector<TMap, TVectorKeys> &&
        CMapValuesAreConvertibleFrom<TMap, TDefaultValue>
    static auto at(const TMap& map, const TVectorKeys& keys, const TDefaultValue& def)
    {
        // not necessarily the same as TDefaultValue, e.g., if we pass in nullptr
        using TValue = typename TMap::mapped_type;

        // since a default value is provided, we don't require all keys to be present
        return toVector<TValue>(keys | std::views::transform([&map, &def](const auto& key) { return containsKey(map, key) ? map.at(key) : def; }));
    }

    // We use TKey&& and TValue&& because we want to preserve and forward the const-ness and rvalue-ness of key and value.
    template <typename TMap, typename TKey, typename TValue> requires
        CMapKeysAreConvertibleFrom<TMap, TKey> &&
        CMapValuesAreConvertibleFrom<TMap, TValue>
    static void insert(TMap& map, TKey&& key, TValue&& value)
    {
        auto [itr, success] = map.insert({std::forward<TKey>(key), std::forward<TValue>(value)});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }

    // An std::initializer_list type (e.g., std::initializer_list<int>) can't be inferred as a template parameter.
    // Only the value type of the initializer list can be inferred (e.g., the int type in std::initializer_list<int>),
    // and only when the initializer list passed to the templated function is non-empty. So this insert(...)
    // specialization is required handle situations where a caller passes in a non-empty initializer list.

    // We use TKey&& because we want to preserve and forward the const-ness and rvalue-ness of key and value.
    template <typename TMap, typename TKey, typename TInitializerListValue> requires
        CMapKeysAreConvertibleFrom<TMap, TKey> &&
        CMapValuesAreConvertibleFromInitializerList<TMap, TInitializerListValue>
    static void insert(TMap& map, TKey&& key, std::initializer_list<TInitializerListValue> initializer_list)
    {
        auto [itr, success] = map.insert({std::forward<TKey>(key), initializer_list});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }

    // This insert(...) specialization is required to handle situations where a caller passes in an empty initializer
    // list. Perhaps surprisingly, this specialization behaves as expected even though EmptyInitializerList is a
    // private class.
private:
    class EmptyInitializerList;
public:
    // We use TKey&& because we want to preserve and forward the const-ness and rvalue-ness of key and value.
    template <typename TMap, typename TKey> requires
        CMapKeysAreConvertibleFrom<TMap, TKey> &&
        CMapValuesAreConvertibleFromEmptyInitializerList<TMap>
    static void insert(TMap& map, TKey&& key, std::initializer_list<EmptyInitializerList> initializer_list)
    {
        auto [itr, success] = map.insert({std::forward<TKey>(key), {}});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }

    template <typename TDestMap, typename TSrcMap> requires
        CMapKeysAndValuesAreConvertibleFromMap<TDestMap, TSrcMap>
    static void insert(TDestMap& dest_map, const TSrcMap& src_map)
    {
        using TKey = typename TDestMap::key_type;

        // assert if there are duplicate keys
        std::vector<TKey> keys_set_intersection;
        std::ranges::set_intersection(keys(dest_map), keys(src_map), std::back_inserter(keys_set_intersection));
        SP_ASSERT(keys_set_intersection.empty());

        dest_map.insert(std::ranges::begin(src_map), std::ranges::end(src_map));
    }

    template <typename TMap, typename TKey> requires
        CMapKeysAreConvertibleFrom<TMap, TKey>
    static void remove(TMap& map, const TKey& key)
    {
        int num_elements_removed = map.erase(key);
        SP_ASSERT(num_elements_removed == 1);
    }

    template <typename TMap, typename TVectorKeys> requires
        CMapKeysAreConvertibleFromVector<TMap, TVectorKeys> &&
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
    // Set (e.g., std::set) functions
    //

    template <typename TSet, typename TKey> requires
        CSetKeysAreConvertibleFrom<TSet, TKey>
    static void remove(TSet& set, const TKey& key)
    {
        SP_ASSERT(set.contains(key));
        set.erase(key);
    }

    template <typename TSet, typename TVectorKeys> requires
        CSetKeysAreConvertibleFromVector<TSet, TVectorKeys> &&
        CVector<TVectorKeys>
    static void remove(TSet& set, const TVectorKeys& keys)
    {
        for (auto& key : keys) {
            remove(set, key);
        }
    }

    template <typename TSet> requires
        CSet<TSet>
    static auto keys(TSet& set)
    {
        using TKey = typename TSet::key_type;

        return toVector<TKey>(set);
    }

    //
    // Functions for (as safely as possible) reinterpreting ranges, vectors, and initializer lists
    //

    template <typename TValue>
    static bool isPtrSufficientlyAlignedFor(void* ptr)
    {
        return isPtrSufficientlyAligned(ptr, alignof(TValue));
    }

    static bool isPtrSufficientlyAligned(void* ptr, size_t alignment_num_bytes)
    {
        return reinterpret_cast<std::uintptr_t>(ptr) % alignment_num_bytes == 0;
    }

    // Reinterpret as span

    // We use TSrcVector&& because want to preserve and forward the const-ness and rvalue-ness of src. We do
    // this to enforce the constraint that if TSrcVector is const, then TDestValue also needs to be const.
    template <typename TDestValue, typename TSrcVector> requires
        CVector<std::remove_cvref_t<TSrcVector>> &&
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
        SP_ASSERT(isPtrSufficientlyAlignedFor<TDestValue>(src_data));

        std::span<TDestValue> dest;
        if (src_num_elements > 0) {
            uint64_t src_num_bytes = src_num_elements*sizeof(TSrcValue);
            SP_ASSERT(src_num_bytes % sizeof(TDestValue) == 0);
            uint64_t dest_num_elements = src_num_bytes / sizeof(TDestValue);
            dest = std::span<TDestValue>(reinterpret_cast<TDestValue*>(src_data), dest_num_elements); // undefined behavior when dest is accessed
        }
        return dest;
    }

    // Reinterpret as vector

    template <typename TDestValue, typename TSrcValue, typename... TSrcTraits>
    static auto reinterpretAsVectorOf(std::vector<TSrcValue, TSrcTraits...>&& src)
    {
        // This function is unsafe because it relies on undefined behavior and the private memory layout of std::vector.

        using TSrcVector = std::vector<TSrcValue, TSrcTraits...>;
        using TSrcAllocator = typename TSrcVector::allocator_type;
        using TDestVector = std::vector<TDestValue, typename std::allocator_traits<TSrcAllocator>::template rebind_alloc<TDestValue>>;

        TDestVector dest;

        // check to make sure that calling delete[] on src and dest data pointers won't invoke any destructors
        SP_ASSERT(std::is_trivially_constructible<TSrcValue>());
        SP_ASSERT(std::is_trivially_constructible<TDestValue>());

        // check that the size of std::vector is the size of exactly 3 pointers stored contiguously
        SP_ASSERT(sizeof(std::vector<TSrcValue>)  == 3*sizeof(TSrcValue*));
        SP_ASSERT(sizeof(std::vector<TDestValue>) == 3*sizeof(TDestValue*));

        TSrcValue** src_ptr = reinterpret_cast<TSrcValue**>(&src);
        TDestValue** dest_ptr = reinterpret_cast<TDestValue**>(&dest);

        TSrcValue* src_begin_ptr    = *(src_ptr + 0);    // undefined behavior
        TSrcValue* src_end_ptr      = *(src_ptr + 1);    // undefined behavior
        TSrcValue* src_capacity_ptr = *(src_ptr + 2);    // undefined behavior

        TDestValue* dest_begin_ptr    = *(dest_ptr + 0); // undefined behavior
        TDestValue* dest_end_ptr      = *(dest_ptr + 1); // undefined behavior
        TDestValue* dest_capacity_ptr = *(dest_ptr + 2); // undefined behavior

        // check to make sure that we're interpreting the layouts of src and dest correctly

        SP_ASSERT(src_begin_ptr    == src.data());
        SP_ASSERT(src_end_ptr      == src_begin_ptr + src.size());
        SP_ASSERT(src_capacity_ptr == src_begin_ptr + src.capacity());

        SP_ASSERT(dest_begin_ptr    == dest.data());
        SP_ASSERT(dest_end_ptr      == dest_begin_ptr + dest.size());
        SP_ASSERT(dest_capacity_ptr == dest_begin_ptr + dest.capacity());

        // check to make sure each pointer is sufficiently aligned for its swapped type

        SP_ASSERT(isPtrSufficientlyAlignedFor<TDestValue>(src_begin_ptr));
        SP_ASSERT(isPtrSufficientlyAlignedFor<TDestValue>(src_end_ptr));
        SP_ASSERT(isPtrSufficientlyAlignedFor<TDestValue>(src_capacity_ptr));

        SP_ASSERT(isPtrSufficientlyAlignedFor<TSrcValue>(dest_begin_ptr));
        SP_ASSERT(isPtrSufficientlyAlignedFor<TSrcValue>(dest_end_ptr));
        SP_ASSERT(isPtrSufficientlyAlignedFor<TSrcValue>(dest_capacity_ptr));

        // swap pointers

        *(src_ptr + 0) = reinterpret_cast<TSrcValue*>(dest_begin_ptr);     // undefined behavior
        *(src_ptr + 1) = reinterpret_cast<TSrcValue*>(dest_end_ptr);       // undefined behavior
        *(src_ptr + 2) = reinterpret_cast<TSrcValue*>(dest_capacity_ptr);  // undefined behavior

        *(dest_ptr + 0) = reinterpret_cast<TDestValue*>(src_begin_ptr);    // undefined behavior
        *(dest_ptr + 1) = reinterpret_cast<TDestValue*>(src_end_ptr);      // undefined behavior
        *(dest_ptr + 2) = reinterpret_cast<TDestValue*>(src_capacity_ptr); // undefined behavior

        SP_ASSERT(dest_begin_ptr == reinterpret_cast<TDestValue*>(src.data()));
        SP_ASSERT(src_begin_ptr  == reinterpret_cast<TSrcValue*>(dest.data()));

        return dest;
    }

    template <typename TDestValue, typename TSrcVector> requires
        CVector<TSrcVector>
    static std::vector<TDestValue> reinterpretAsVectorOf(const TSrcVector& src)
    {
        return reinterpretAsVectorImpl<std::vector<TDestValue>>(src.data(), src.size());
    }

    template <typename TDestValue, typename TDestAllocator, typename TSrcVector> requires
        CVector<TSrcVector>
    static std::vector<TDestValue, TDestAllocator> reinterpretAsVectorOf(const TSrcVector& src)
    {
        return reinterpretAsVectorImpl<std::vector<TDestValue, TDestAllocator>>(src.data(), src.size());
    }

    // Don't infer TSrcValue from the input in the functions below, because we want to force the user to
    // explicitly specify the intended value type of the input somewhere. If we allow the value type of
    // the input to be inferred automatically, we create a situation where, e.g., the user wants to create
    // an initializer list of floats, but accidentally creates an initializer list of doubles. We don't need
    // this extra layer of safety for reinterpretAsVectorOf(...), because the input to that function is,
    // e.g., an std::vector, where the user would have already specified its value type somewhere in their
    // code.

    template <typename TDestValue, typename TSrcValue, typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, TSrcValue>
    static std::vector<TDestValue> reinterpretAsVector(TRange&& range)
    {
        return reinterpretAsVectorImpl<std::vector<TDestValue>, TSrcValue>(std::forward<TRange>(range));
    }

    template <typename TDestValue, typename TDestAllocator, typename TSrcValue, typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, TSrcValue>
    static std::vector<TDestValue, TDestAllocator> reinterpretAsVector(TRange&& range)
    {
        return reinterpretAsVectorImpl<std::vector<TDestValue, TDestAllocator>, TSrcValue>(std::forward<TRange>(range));
    }

    template <typename TDestValue, typename TSrcValue, typename TSrcInitializerListValue> requires
        std::same_as<TSrcValue, TSrcInitializerListValue>
    static std::vector<TDestValue> reinterpretAsVector(std::initializer_list<TSrcInitializerListValue> src)
    {
        return reinterpretAsVectorImpl<std::vector<TDestValue>>(std::ranges::data(src), std::ranges::size(src));
    }

    template <typename TDestValue, typename TDestAllocator, typename TSrcValue, typename TSrcInitializerListValue> requires
        std::same_as<TSrcValue, TSrcInitializerListValue>
    static std::vector<TDestValue, TDestAllocator> reinterpretAsVector(std::initializer_list<TSrcInitializerListValue> src)
    {
        return reinterpretAsVectorImpl<std::vector<TDestValue, TDestAllocator>>(std::ranges::data(src), std::ranges::size(src));
    }

    // We can infer TSrcValue below because there is no potential for ambiguity.

    template <typename TDestValue, typename TSrcValue>
    static std::vector<TDestValue> reinterpretAsVector(const TSrcValue* src_data, uint64_t src_num_elements)
    {
        return reinterpretAsVectorImpl<std::vector<TDestValue>>(src_data, src_num_elements);
    }

    template <typename TDestValue, typename TDestAllocator, typename TSrcValue>
    static std::vector<TDestValue, TDestAllocator> reinterpretAsVector(const TSrcValue* src_data, uint64_t src_num_elements)
    {
        return reinterpretAsVectorImpl<std::vector<TDestValue, TDestAllocator>>(src_data, src_num_elements);
    }

private:

    // Low-level helpers that require TDestVector (e.g., std::vector<int>) instead of TDestValue (e.g., int)
    // as a template parameter.

    template <typename TDestVector, typename TSrcValue, typename TRange> requires
        CVector<TDestVector> &&
        CRangeValuesAreConvertibleTo<TRange, TSrcValue>
    static TDestVector reinterpretAsVectorImpl(TRange&& range)
    {
        // This function is unsafe because it relies on undefined behavior.

        using TDestValue = TDestVector::value_type;

        TDestVector dest;

        size_t num_src_elements_per_resize;
        size_t num_dest_elements_per_resize;
        if (sizeof(TSrcValue) < sizeof(TDestValue)) {
            SP_ASSERT(sizeof(TDestValue) % sizeof(TSrcValue) == 0);
            num_src_elements_per_resize  = sizeof(TDestValue) / sizeof(TSrcValue);
            num_dest_elements_per_resize = 1;
        } else if (sizeof(TSrcValue) == sizeof(TDestValue)) {
            num_src_elements_per_resize  = 1;
            num_dest_elements_per_resize = 1;
        } else {
            SP_ASSERT(sizeof(TSrcValue) % sizeof(TDestValue) == 0);
            num_src_elements_per_resize  = 1;
            num_dest_elements_per_resize = sizeof(TSrcValue) / sizeof(TDestValue);
        }

        size_t src_index_within_dest_element = 0;
        for (auto& range_value : range) {
            if (src_index_within_dest_element == 0) {
                resizeUninitialized(dest, dest.size() + num_dest_elements_per_resize);
            }
            TDestValue* dest_ptr = &(dest.at(dest.size() - num_dest_elements_per_resize)); // get ptr after resize because data might have moved
            SP_ASSERT(dest_ptr);
            SP_ASSERT(isPtrSufficientlyAlignedFor<TDestValue>(dest_ptr));
            TSrcValue* src_ptr = reinterpret_cast<TSrcValue*>(dest_ptr) + src_index_within_dest_element;
            SP_ASSERT(isPtrSufficientlyAlignedFor<TSrcValue>(src_ptr));
            *src_ptr = range_value;                                                        // undefined behavior
            src_index_within_dest_element += 1;
            if (src_index_within_dest_element == num_src_elements_per_resize) {
                src_index_within_dest_element = 0;
            }
        }
        SP_ASSERT(src_index_within_dest_element == 0);

        return dest;
    }

    template <typename TDestVector, typename TSrcValue> requires
        CVector<TDestVector>
    static TDestVector reinterpretAsVectorImpl(const TSrcValue* src_data, uint64_t src_num_elements)
    {
        using TDestValue = TDestVector::value_type;

        TDestVector dest;
        if (src_num_elements > 0) {
            uint64_t src_num_bytes = src_num_elements*sizeof(TSrcValue);
            SP_ASSERT(src_num_bytes % sizeof(TDestValue) == 0);
            uint64_t dest_num_elements = src_num_bytes / sizeof(TDestValue);
            resizeUninitialized(dest, dest_num_elements);
            std::memcpy(dest.data(), src_data, src_num_bytes);
        }
        return dest;
    }
};
