//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <algorithm> // std::ranges::sort
#include <concepts>  // std::convertible_to, std::same_as
#include <memory>    // std::allocator_traits
#include <ranges>    // std::ranges::contiguous_range, std::ranges::sized_range, std::views::transform
#include <vector>

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
    static void resizeUninitialized(std::vector<TValue, TVectorTraits...>& vector, int size)
    {
        struct TValueNoDefaultInit { TValue value; TValueNoDefaultInit() {} };

        using TVector = std::vector<TValue, TVectorTraits...>;
        using TAllocator = typename TVector::allocator_type;
        using TVectorNoDefaultInit = std::vector<TValueNoDefaultInit, typename std::allocator_traits<TAllocator>::template rebind_alloc<TValueNoDefaultInit>>;

        SP_ASSERT(sizeof(TValue)    == sizeof(TValueNoDefaultInit));
        SP_ASSERT(sizeof(TValue[2]) == sizeof(TValueNoDefaultInit[2]));
        SP_ASSERT(sizeof(TValue[4]) == sizeof(TValueNoDefaultInit[4]));

        TVectorNoDefaultInit* vector_no_default_init = reinterpret_cast<TVectorNoDefaultInit*>(&vector);
        vector_no_default_init->resize(size);

        SP_ASSERT(vector.size() == size);
    }

    static bool isPtrSufficientlyAligned(void* ptr, size_t alignment_num_bytes)
    {
        return reinterpret_cast<std::uintptr_t>(ptr) % alignment_num_bytes == 0;
    }
};
