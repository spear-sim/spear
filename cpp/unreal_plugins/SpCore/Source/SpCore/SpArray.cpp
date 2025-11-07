//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpArray.h"

#include <stdint.h> // int64_t, uint64_t

#include <initializer_list>
#include <map>
#include <ranges>  // std::views::filter, std::views::transform
#include <string>
#include <utility> // std::make_pair, std::move
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

//
// SpArrayShapeUtils
//

std::vector<uint64_t> SpArrayShapeUtils::getShape(const std::vector<int64_t>& shape, uint64_t num_elements)
{
    SP_ASSERT(Std::toVector<uint64_t>(shape | std::views::filter([](auto s) { return s < -1; })).size() == 0); // must contain no entries less than -1
    SP_ASSERT(Std::toVector<uint64_t>(shape | std::views::filter([](auto s) { return s == -1; })).size() <= 1); // must contain at most one -1

    // If shape is {}, then 0 is the only value for num_elements that matches shape, and we should return {}.
    if (shape.empty()) {
        SP_ASSERT(num_elements == 0);
        return {};
    }

    // If shape doesn't contain a wildcard, then return shape.
    if (!Std::contains(shape, -1)) {
        return Std::reinterpretAsVectorOf<uint64_t>(shape);
    }

    std::vector<uint64_t> shape_non_wildcard = Std::toVector<uint64_t>(shape | std::views::filter([](auto s) { return s >= 0; }));

    // If we get an empty list after removing all -1 values from shape, then we know that shape is {-1}, so return {num_elements}.
    if (shape_non_wildcard.empty()) {
        return {num_elements};
    }

    // We know shape_non_wildcard is non-empty here, so we can compute the total number of elements
    // implied by non-wildcard values by accumulating.
    uint64_t num_elements_non_wildcard = std::accumulate(shape_non_wildcard.begin(), shape_non_wildcard.end(), 1, std::multiplies<uint64_t>());

    // If num_elements_non_wildcard is 0, then 0 is the only value for num_elements that matches
    // shape, and we can choose any wildcard value we want, so we choose 0 for simplicity. Otherwise,
    // num_elements must divide evenly by num_elements_non_wildcard, and we choose this ratio to be
    // to be the wildcard value.
    uint64_t wildcard_value;
    if (num_elements_non_wildcard == 0) {
        SP_ASSERT(num_elements == 0);
        wildcard_value = 0;
    } else {
        SP_ASSERT(num_elements % num_elements_non_wildcard == 0);
        wildcard_value = num_elements / num_elements_non_wildcard;
    }

    return Std::toVector<uint64_t>(shape | std::views::transform([wildcard_value](auto s) { return s == -1 ? wildcard_value : s; }));
}

//
// SpPackedArray
//

void SpPackedArray::resolve()
{
    // validate internal state
    SP_ASSERT(!view_);
    SP_ASSERT(data_source_ == SpArrayDataSource::Internal);
    
    if (!data_.empty()) {
        view_ = data_.data();
    }
}

void SpPackedArray::resolve(const SpArraySharedMemoryView& shared_memory_view)
{
    // validate internal state
    SP_ASSERT(!view_);
    SP_ASSERT(data_source_ == SpArrayDataSource::Shared);

    uint64_t num_elements = 0;
    if (!shape_.empty()) {
        num_elements = std::accumulate(shape_.begin(), shape_.end(), 1, std::multiplies<uint64_t>());
    }

    // validate consistency of internal state and shared memory state
    SP_ASSERT(num_elements * SpArrayDataTypeUtils::getSizeOf(data_type_) <= shared_memory_view.num_bytes_);

    // update internal state
    if (num_elements > 0) {
        view_ = shared_memory_view.data_;
    }
    shared_memory_usage_flags_ = shared_memory_view.usage_flags_;
}

void SpPackedArray::validate(SpArraySharedMemoryUsageFlags usage_flags) const
{
    SP_ASSERT(data_type_ != SpArrayDataType::Invalid);

    uint64_t num_elements = 0;
    if (!shape_.empty()) {
        num_elements = std::accumulate(shape_.begin(), shape_.end(), 1, std::multiplies<uint64_t>());
    }

    switch (data_source_) {
        case SpArrayDataSource::Internal:
            SP_ASSERT(num_elements*SpArrayDataTypeUtils::getSizeOf(data_type_) == data_.size());
            SP_ASSERT(shared_memory_name_ == "");
            SP_ASSERT(shared_memory_usage_flags_ == SpArraySharedMemoryUsageFlags::DoNotUse);

            if (data_.empty()) {
                SP_ASSERT(!view_);
            } else {
                SP_ASSERT(view_ == data_.data());
            }

            break;

        case SpArrayDataSource::External:
            SP_ASSERT(data_.empty());
            SP_ASSERT(shared_memory_name_ == "");
            SP_ASSERT(shared_memory_usage_flags_ == SpArraySharedMemoryUsageFlags::DoNotUse);

            if (num_elements > 0) {
                SP_ASSERT(!view_);
            } else {
                SP_ASSERT(view_);
            }

            break;

        case SpArrayDataSource::Shared:
            SP_ASSERT(data_.empty());
            SP_ASSERT(shared_memory_name_ != "");
            SP_ASSERT(shared_memory_usage_flags_ & usage_flags);

            if (num_elements == 0) {
                SP_ASSERT(!view_);
            } else {
                SP_ASSERT(view_);
            }

            break;

        case SpArrayDataSource::Invalid:
            SP_ASSERT(false);
            break;

        default:
            SP_ASSERT(false);
            break;
    }
}

//
// SpArrayUtils
//

//
// typically called at the beginning of a C++ entry point to prepare items for use (e.g., by resolving pointers to shared memory)
//

void SpArrayUtils::resolve(std::vector<SpPackedArray>& packed_arrays, const std::map<std::string, SpArraySharedMemoryView>& shared_memory_views)
{
    for (auto& packed_array : packed_arrays) {
        resolve(packed_array, shared_memory_views);
    }
}

void SpArrayUtils::resolve(std::map<std::string, SpPackedArray>& packed_arrays, const std::map<std::string, SpArraySharedMemoryView>& shared_memory_views)
{
    for (auto& [name, packed_array] : packed_arrays) {
        resolve(packed_array, shared_memory_views);
    }
}

void SpArrayUtils::resolve(SpPackedArray& packed_array, const std::map<std::string, SpArraySharedMemoryView>& shared_memory_views)
{
    if (packed_array.data_source_ == SpArrayDataSource::Internal) {
        packed_array.resolve();
    } else if (packed_array.data_source_ == SpArrayDataSource::Shared) {
        SP_ASSERT(packed_array.shared_memory_name_ != "");
        SP_ASSERT(Std::containsKey(shared_memory_views, packed_array.shared_memory_name_));
        packed_array.resolve(shared_memory_views.at(packed_array.shared_memory_name_));
    }
}

//
// typically called after resolve(...) to validate the internal consistency of packed arrays before they are used in an SpFunc
//

void SpArrayUtils::validate(const std::vector<SpPackedArray>& packed_arrays, SpArraySharedMemoryUsageFlags usage_flags)
{
    for (auto& packed_array : packed_arrays) {
        validate(packed_array, usage_flags);
    }
}

void SpArrayUtils::validate(const std::map<std::string, SpPackedArray>& packed_arrays, SpArraySharedMemoryUsageFlags usage_flags)
{
    for (auto& [name, packed_array] : packed_arrays) {
        validate(packed_array, usage_flags);
    }
}

void SpArrayUtils::validate(const SpPackedArray& packed_array, SpArraySharedMemoryUsageFlags usage_flags)
{
    packed_array.validate(usage_flags);
}

//
// typically called before calling an SpFunc to set args, and from inside an SpFunc to set return values
//

std::map<std::string, SpPackedArray> SpArrayUtils::moveToPackedArrays(std::initializer_list<SpArrayBase*> arrays)
{
    // convert from initializer_list to vector
    return moveToPackedArrays(Std::toVector<SpArrayBase*>(arrays));
}

std::map<std::string, SpPackedArray> SpArrayUtils::moveToPackedArrays(const std::vector<SpArrayBase*>& arrays)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(arrays, nullptr));
    auto array_map = Std::toMap<std::string, SpArrayBase*>(
        arrays | std::views::transform([](auto array) { return std::make_pair(array->getName(), array); }));
    return moveToPackedArrays(array_map);
}

std::map<std::string, SpPackedArray> SpArrayUtils::moveToPackedArrays(const std::map<std::string, SpArrayBase*>& arrays)
{
    // input is a map from names to SpArray pointers, output is a map from names to SpPackedArrays
    std::map<std::string, SpPackedArray> packed_arrays;
    for (auto& [name, array] : arrays) {
        SpPackedArray packed_array;
        array->moveToPackedArray(packed_array);
        Std::insert(packed_arrays, std::move(name), std::move(packed_array));
    }
    return packed_arrays;
}

//
// typically called from inside an SpFunc to get args, and after calling an SpFunc to get return values
//

void SpArrayUtils::moveFromPackedArrays(std::initializer_list<SpArrayBase*> arrays, std::map<std::string, SpPackedArray>& packed_arrays)
{
    // convert from initializer_list to vector
    moveFromPackedArrays(Std::toVector<SpArrayBase*>(arrays), packed_arrays);
}

void SpArrayUtils::moveFromPackedArrays(const std::vector<SpArrayBase*>& arrays, std::map<std::string, SpPackedArray>& packed_arrays)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(arrays, nullptr));
    auto array_map = Std::toMap<std::string, SpArrayBase*>(
        arrays | std::views::transform([](auto array) { return std::make_pair(array->getName(), array); }));
    moveFromPackedArrays(array_map, packed_arrays);
}

void SpArrayUtils::moveFromPackedArrays(const std::map<std::string, SpArrayBase*>& arrays, std::map<std::string, SpPackedArray>& packed_arrays)
{
    for (auto& [name, array] : arrays) {
        SP_ASSERT(Std::containsKey(packed_arrays, name));
        array->moveFromPackedArray(packed_arrays.at(name));
    }
}

//
// typically called from inside an SpFunc to get args, and after calling an SpFunc to get return values
//

void SpArrayUtils::setViews(std::initializer_list<SpArrayViewBase*> array_views, const std::map<std::string, SpPackedArray>& packed_arrays)
{
    // convert from initializer_list to vector
    setViews(Std::toVector<SpArrayViewBase*>(array_views), packed_arrays);
}

void SpArrayUtils::setViews(const std::vector<SpArrayViewBase*>& array_views, const std::map<std::string, SpPackedArray>& packed_arrays)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(array_views, nullptr));
    auto array_view_map = Std::toMap<std::string, SpArrayViewBase*>(
        array_views | std::views::transform([](auto array_view) { return std::make_pair(array_view->getName(), array_view); }));
    setViews(array_view_map, packed_arrays);
}

void SpArrayUtils::setViews(const std::map<std::string, SpArrayViewBase*>& array_views, const std::map<std::string, SpPackedArray>& packed_arrays)
{
    for (auto& [name, array_view] : array_views) {
        SP_ASSERT(Std::containsKey(packed_arrays, name));
        array_view->setView(packed_arrays.at(name));
    }
}
