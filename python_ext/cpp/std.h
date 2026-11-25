//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // uint8_t, uint64_t, uintptr_t

#include <algorithm>   // std::ranges::copy, std::ranges::sort, std::ranges::unique
#include <concepts>    // std::convertible_to, std::same_as
#include <iterator>    // std::back_inserter
#include <map>
#include <memory>      // std::allocator_traits
#include <ranges>      // std::ranges::begin, std::ranges::contiguous_range, std::ranges::end, std::ranges::range, std::ranges::sized_range,
                       // std::views::transform
#include <type_traits> // std::is_empty
#include <utility>     // std::forward, std::pair
#include <vector>

#include "assert.h"

template <typename TRange, typename TValue>
concept CRangeValuesAreConvertibleTo =
    std::ranges::range<TRange> &&
    requires (TRange range) {
        { *(std::ranges::begin(range)) } -> std::convertible_to<const TValue&>;
        { *(std::ranges::end(range)) } -> std::convertible_to<const TValue&>;
    };

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

class Std
{
public:
    Std() = delete;
    ~Std() = delete;

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

    template <typename TValue, typename... TVectorTraits>
    static void resizeUninitialized(std::vector<TValue, TVectorTraits...>& vector, uint64_t size)
    {
        // This function is unsafe because it relies on undefined behavior when invoking vector_no_default_init->resize().

        struct alignas(alignof(TValue)) TValueNoDefaultInit { uint8_t value[sizeof(TValue)]; TValueNoDefaultInit() = default; };

        using TVector = std::vector<TValue, TVectorTraits...>;
        using TAllocator = typename TVector::allocator_type;
        using TVectorNoDefaultInit = std::vector<TValueNoDefaultInit, typename std::allocator_traits<TAllocator>::template rebind_alloc<TValueNoDefaultInit>>;

        SP_ASSERT(sizeof(TValue) == sizeof(TValueNoDefaultInit));
        SP_ASSERT(alignof(TValue) == alignof(TValueNoDefaultInit));

        // check to make sure the allocator is stateless and can be used to deallocate memory from a different instance 
        SP_ASSERT(std::is_empty<TAllocator>());
        SP_ASSERT(std::allocator_traits<TAllocator>::is_always_equal::value);

        TVectorNoDefaultInit* vector_no_default_init = reinterpret_cast<TVectorNoDefaultInit*>(&vector);
        vector_no_default_init->resize(size); // undefined behavior

        SP_ASSERT(vector.size() == size);
    }

    static bool isPtrSufficientlyAligned(const void* ptr, size_t alignment_num_bytes)
    {
        return reinterpret_cast<std::uintptr_t>(ptr) % alignment_num_bytes == 0;
    }
};
