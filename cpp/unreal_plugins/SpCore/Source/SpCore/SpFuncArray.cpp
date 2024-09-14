//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpFuncArray.h"

#include <stddef.h> // uint64_t

#include <initializer_list>
#include <map>
#include <string>
#include <utility> // std::make_pair
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

//
// SpFuncPackedArray
//

void SpFuncPackedArray::setView(const SpFuncSharedMemoryView& shared_memory_view)
{
    // validate internal state
    SP_ASSERT(!view_);
    SP_ASSERT(data_source_ == SpFuncArrayDataSource::Shared);

    // validate consistency of internal state and shared memory state
    uint64_t num_elements = 0;
    if (!shape_.empty()) {
        num_elements = std::accumulate(shape_.begin(), shape_.end(), 1, std::multiplies<uint64_t>());
    }
    SP_ASSERT(num_elements*SpFuncArrayDataTypeUtils::getSizeOf(data_type_) <= shared_memory_view.num_bytes_);

    // update internal state
    view_ = shared_memory_view.data_;
    shared_memory_usage_flags_ = shared_memory_view.usage_flags_;
}

void SpFuncPackedArray::validate(SpFuncSharedMemoryUsageFlags usage_flags) const
{
    SP_ASSERT(data_type_ != SpFuncArrayDataType::Invalid);

    uint64_t num_elements = 0;
    if (!shape_.empty()) {
        num_elements = std::accumulate(shape_.begin(), shape_.end(), 1, std::multiplies<uint64_t>());
    }

    switch (data_source_) {
        case SpFuncArrayDataSource::Internal:
            SP_ASSERT(view_ == data_.data());
            SP_ASSERT(num_elements*SpFuncArrayDataTypeUtils::getSizeOf(data_type_) == data_.size());
            SP_ASSERT(shared_memory_name_ == "");
            break;

        case SpFuncArrayDataSource::External:
            SP_ASSERT(data_.empty());
            SP_ASSERT(num_elements == 0 || view_);
            SP_ASSERT(shared_memory_name_ == "");
            break;

        case SpFuncArrayDataSource::Shared:
            SP_ASSERT(data_.empty());
            SP_ASSERT(num_elements == 0 || view_);
            SP_ASSERT(shared_memory_name_ != "");
            SP_ASSERT(shared_memory_usage_flags_ & usage_flags);
            break;

        case SpFuncArrayDataSource::Invalid:
            SP_ASSERT(false);
            break;

        default:
            SP_ASSERT(false);
            break;
    }
}

//
// SpFuncArrayUtils
//

//
// typically called at the beginning of a C++ entry point to prepare items for use (e.g., by resolving pointers to shared memory)
//

void SpFuncArrayUtils::resolve(SpFuncPackedArray& packed_array, const std::map<std::string, SpFuncSharedMemoryView>& shared_memory_views)
{
    if (packed_array.data_source_ == SpFuncArrayDataSource::Shared) {
        SP_ASSERT(packed_array.shared_memory_name_ != "");
        SP_ASSERT(Std::containsKey(shared_memory_views, packed_array.shared_memory_name_));
        packed_array.setView(shared_memory_views.at(packed_array.shared_memory_name_));
    }
}

void SpFuncArrayUtils::resolve(std::vector<SpFuncPackedArray>& packed_arrays, const std::map<std::string, SpFuncSharedMemoryView>& shared_memory_views)
{
    for (auto& packed_array : packed_arrays) {
        resolve(packed_array, shared_memory_views);
    }
}

void SpFuncArrayUtils::resolve(std::map<std::string, SpFuncPackedArray>& packed_arrays, const std::map<std::string, SpFuncSharedMemoryView>& shared_memory_views)
{
    for (auto& [name, packed_array] : packed_arrays) {
        resolve(packed_array, shared_memory_views);
    }
}

//
// typically called after resolve(...) to validate the internal consistency of packed arrays before they are used in an SpFunc
//

void SpFuncArrayUtils::validate(const SpFuncPackedArray& packed_array, SpFuncSharedMemoryUsageFlags usage_flags)
{
    packed_array.validate(usage_flags);
}

void SpFuncArrayUtils::validate(const std::vector<SpFuncPackedArray>& packed_arrays, SpFuncSharedMemoryUsageFlags usage_flags)
{
    for (auto& packed_array : packed_arrays) {
        validate(packed_array, usage_flags);
    }
}

void SpFuncArrayUtils::validate(const std::map<std::string, SpFuncPackedArray>& packed_arrays, SpFuncSharedMemoryUsageFlags usage_flags)
{
    for (auto& [name, packed_array] : packed_arrays) {
        validate(packed_array, usage_flags);
    }
}

//
// typically called before calling an SpFunc to set args, and from inside an SpFunc to set return values
//

std::map<std::string, SpFuncPackedArray> SpFuncArrayUtils::moveToPackedArrays(std::initializer_list<SpFuncArrayBase*> arrays)
{
    // convert from initializer_list to vector
    return moveToPackedArrays(Std::toVector<SpFuncArrayBase*>(arrays));
}

std::map<std::string, SpFuncPackedArray> SpFuncArrayUtils::moveToPackedArrays(const std::vector<SpFuncArrayBase*>& arrays)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(arrays, nullptr));
    auto array_map = Std::toMap<std::string, SpFuncArrayBase*>(
        arrays | std::views::transform([](auto array) { return std::make_pair(array->getName(), array); }));
    return moveToPackedArrays(array_map);
}

std::map<std::string, SpFuncPackedArray> SpFuncArrayUtils::moveToPackedArrays(const std::map<std::string, SpFuncArrayBase*>& arrays)
{
    // input is a map from names to SpFuncArray pointers, output is a map from names to SpFuncPackedArrays
    std::map<std::string, SpFuncPackedArray> packed_arrays;
    for (auto& [name, array] : arrays) {
        SpFuncPackedArray packed_array;
        array->moveToPackedArray(packed_array);
        Std::insert(packed_arrays, name, std::move(packed_array));
    }
    return packed_arrays;
}

//
// typically called from inside an SpFunc to get args, and after calling an SpFunc to get return values
//

void SpFuncArrayUtils::moveFromPackedArrays(std::initializer_list<SpFuncArrayBase*> arrays, std::map<std::string, SpFuncPackedArray>& packed_arrays)
{
    // convert from initializer_list to vector
    moveFromPackedArrays(Std::toVector<SpFuncArrayBase*>(arrays), packed_arrays);
}

void SpFuncArrayUtils::moveFromPackedArrays(const std::vector<SpFuncArrayBase*>& arrays, std::map<std::string, SpFuncPackedArray>& packed_arrays)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(arrays, nullptr));
    auto array_map = Std::toMap<std::string, SpFuncArrayBase*>(
        arrays | std::views::transform([](auto array) { return std::make_pair(array->getName(), array); }));
    moveFromPackedArrays(array_map, packed_arrays);
}

void SpFuncArrayUtils::moveFromPackedArrays(const std::map<std::string, SpFuncArrayBase*>& arrays, std::map<std::string, SpFuncPackedArray>& packed_arrays)
{
    for (auto& [name, array] : arrays) {
        SP_ASSERT(Std::containsKey(packed_arrays, name));
        array->moveFromPackedArray(packed_arrays.at(name));
    }
}

//
// typically called from inside an SpFunc to get args, and after calling an SpFunc to get return values
//

void SpFuncArrayUtils::setViewsFromPackedArrays(std::initializer_list<SpFuncArrayViewBase*> array_views, const std::map<std::string, SpFuncPackedArray>& packed_arrays)
{
    // convert from initializer_list to vector
    setViewsFromPackedArrays(Std::toVector<SpFuncArrayViewBase*>(array_views), packed_arrays);
}

void SpFuncArrayUtils::setViewsFromPackedArrays(const std::vector<SpFuncArrayViewBase*>& array_views, const std::map<std::string, SpFuncPackedArray>& packed_arrays)
{
    // convert from vector to map
    SP_ASSERT(!Std::contains(array_views, nullptr));
    auto array_view_map = Std::toMap<std::string, SpFuncArrayViewBase*>(
        array_views | std::views::transform([](auto array_view) { return std::make_pair(array_view->getName(), array_view); }));
    setViewsFromPackedArrays(array_view_map, packed_arrays);
}

void SpFuncArrayUtils::setViewsFromPackedArrays(const std::map<std::string, SpFuncArrayViewBase*>& array_views, const std::map<std::string, SpFuncPackedArray>& packed_arrays)
{
    for (auto& [name, array_view] : array_views) {
        SP_ASSERT(Std::containsKey(packed_arrays, name));
        array_view->setView(packed_arrays.at(name));
    }
}
