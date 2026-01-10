//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <Containers/Array.h>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

//
// When creating an UnrealArrayUpdateDataPtrScope object, we enforce the constraint that array's existing
// data region must be at least as big as the user's data_ptr region, because we want to guarantee that array
// will not resize itself if the user adds elements that would fit in the user's data region. However, we
// don't test for exact equality between the size of the array's data region and the data_ptr data region.
// This is because calling array.Reserve(num_elements) is allowed to internally allocate more than num_elements
// worth of space. In other words, the array's existing data region might be larger than the user's data
// region, regardless of how its size is set.
//
// Therefore, after UnrealArrayUpdateDataPtrScope has replaced the array's original internal data pointer
// with the user's data pointer, the user must be careful not to add more than num_elements elements to array,
// because if the user adds too many elements, there is nothing to prevent array from writing past the end of
// the user's data region. This is because, as far as array is concerned, if it has internally allocated more
// than num_elements worth of space, then it has enough space to perform the user's requested add operations
// safely. Eventually, if the user adds too many elements, it will trigger a resize operation, in which case
// the array will no longer be corrupting heap memory, but it will also no longer be backed by the user's
// data region.
// 
// So, to avoid corrupting heap memory, and to guarantee that array only writes data to the user's intended
// data region, the user must not add more than num_elements to array when inside an UnrealArrayUpdatePtrScope.
//

template <typename TValue>
class UnrealArrayUpdateDataPtrScope
{
public:
    UnrealArrayUpdateDataPtrScope() = delete;
    UnrealArrayUpdateDataPtrScope(TArray<TValue>& array, void* data_ptr, uint64_t num_bytes) : array_(array)
    {
        num_bytes_ = num_bytes;
        array_data_ptr_ = updateDataPtr(array_, data_ptr, num_bytes_);
    };

    ~UnrealArrayUpdateDataPtrScope()
    {
        updateDataPtr(array_, array_data_ptr_, num_bytes_);
    };

private:
    static TValue* updateDataPtr(TArray<TValue>& array, void* data_ptr, uint64_t num_bytes)
    {
        // This function is unsafe because it relies on undefined behavior and the private memory layout of TArray.

        SP_ASSERT(num_bytes % sizeof(TValue) == 0);
        uint64_t num_elements = num_bytes / sizeof(TValue);

        SP_ASSERT(array.Max() >= 0);
        SP_ASSERT(num_elements <= static_cast<uint64_t>(array.Max()));
        SP_ASSERT(num_bytes <= array.GetAllocatedSize());
        SP_ASSERT(Std::isPtrSufficientlyAlignedFor<TValue>(data_ptr));

        // Get pointer to array object, interpret as a pointer-to-TValue*.
        TValue** array_ptr = reinterpret_cast<TValue**>(&array);
        SP_ASSERT(array_ptr);

        TValue* array_data_ptr = array.GetData();

        // Check that the pointer to the array object, when interpreted as a pointer-to-TValue*, does indeed
        // point to the array's underlying data.
        SP_ASSERT(*array_ptr == array.GetData());    // undefined behavior

        // Update the array's underlying data pointer.
        *array_ptr = static_cast<TValue*>(data_ptr); // undefined behavior
        SP_ASSERT(data_ptr == array.GetData());

        return array_data_ptr;
    };

    TArray<TValue>& array_;
    TValue* array_data_ptr_ = nullptr;
    uint64_t num_bytes_ = 0;
};
