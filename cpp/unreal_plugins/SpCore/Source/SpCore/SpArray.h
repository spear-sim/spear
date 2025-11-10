//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t

#include <initializer_list>
#include <functional> // std::multiplies
#include <map>
#include <numeric>    // std::accumulate
#include <ranges>     // std::ranges::data, std::ranges::size
#include <span>
#include <string>
#include <utility>    // std::move
#include <vector>

#include <HAL/Platform.h>         // SPCORE_API
#include <Math/Float16.h>
#include <Misc/EnumClassFlags.h>  // ENUM_CLASS_FLAGS
#include <UObject/ObjectMacros.h> // UENUM

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpArray.generated.h"

//
// Each SpArray has a shape, and in some cases the shape can be specified with an array that includes a
// single -1 which acts as a wildcard, so we provide a function to compute the complete shape array, given an
// incomplete shape array with a wildcard.
//

class SPCORE_API SpArrayShapeUtils
{
public:
    SpArrayShapeUtils() = delete;
    ~SpArrayShapeUtils() = delete;

    static std::vector<uint64_t> getShape(const std::vector<int64_t>& shape, uint64_t num_elements);
};

//
// Each SpArray has a data type (uint8, float32, etc) and a data source (backed by an internal std::vector,
// backed by an external pointer, backed by shared memory, etc).
//

enum class SpArrayDataType
{
    Invalid = -1,
    UInt8   = 0,
    Int8    = 1,
    UInt16  = 2,
    Int16   = 3,
    UInt32  = 4,
    Int32   = 5,
    UInt64  = 6,
    Int64   = 7,
    Float16 = 8,
    Float32 = 9,
    Float64 = 10
};

UENUM()
enum class ESpArrayDataType
{
    Invalid = Unreal::getConstEnumValue(SpArrayDataType::Invalid),
    UInt8   = Unreal::getConstEnumValue(SpArrayDataType::UInt8),
    Int8    = Unreal::getConstEnumValue(SpArrayDataType::Int8),
    UInt16  = Unreal::getConstEnumValue(SpArrayDataType::UInt16),
    Int16   = Unreal::getConstEnumValue(SpArrayDataType::Int16),
    UInt32  = Unreal::getConstEnumValue(SpArrayDataType::UInt32),
    Int32   = Unreal::getConstEnumValue(SpArrayDataType::Int32),
    UInt64  = Unreal::getConstEnumValue(SpArrayDataType::UInt64),
    Int64   = Unreal::getConstEnumValue(SpArrayDataType::Int64),
    Float16 = Unreal::getConstEnumValue(SpArrayDataType::Float16),
    Float32 = Unreal::getConstEnumValue(SpArrayDataType::Float32),
    Float64 = Unreal::getConstEnumValue(SpArrayDataType::Float64)
};

UENUM()
enum class ESpArrayShortDataType
{
    Invalid = Unreal::getConstEnumValue(SpArrayDataType::Invalid),
    u1      = Unreal::getConstEnumValue(SpArrayDataType::UInt8),
    i1      = Unreal::getConstEnumValue(SpArrayDataType::Int8),
    u2      = Unreal::getConstEnumValue(SpArrayDataType::UInt16),
    i2      = Unreal::getConstEnumValue(SpArrayDataType::Int16),
    u4      = Unreal::getConstEnumValue(SpArrayDataType::UInt32),
    i4      = Unreal::getConstEnumValue(SpArrayDataType::Int32),
    u8      = Unreal::getConstEnumValue(SpArrayDataType::UInt64),
    i8      = Unreal::getConstEnumValue(SpArrayDataType::Int64),
    f2      = Unreal::getConstEnumValue(SpArrayDataType::Float16),
    f4      = Unreal::getConstEnumValue(SpArrayDataType::Float32),
    f8      = Unreal::getConstEnumValue(SpArrayDataType::Float64)
};

enum class SpArrayDataSource
{
    Invalid  = -1,
    Internal = 0,
    External = 1,
    Shared   = 2
};

UENUM()
enum class ESpArrayDataSource
{
    Invalid  = Unreal::getConstEnumValue(SpArrayDataSource::Invalid),
    Internal = Unreal::getConstEnumValue(SpArrayDataSource::Internal),
    External = Unreal::getConstEnumValue(SpArrayDataSource::External),
    Shared   = Unreal::getConstEnumValue(SpArrayDataSource::Shared)
};

class SPCORE_API SpArrayDataTypeUtils
{
public:
    SpArrayDataTypeUtils() = delete;
    ~SpArrayDataTypeUtils() = delete;

    template <typename TValue> static SpArrayDataType getDataType() { SP_LOG_CURRENT_FUNCTION(); SP_ASSERT(false); return SpArrayDataType::Invalid; }
    template <> SpArrayDataType getDataType<uint8_t>()  { return SpArrayDataType::UInt8; }
    template <> SpArrayDataType getDataType<int8_t>()   { return SpArrayDataType::Int8; }
    template <> SpArrayDataType getDataType<uint16_t>() { return SpArrayDataType::UInt16; }
    template <> SpArrayDataType getDataType<int16_t>()  { return SpArrayDataType::Int16; }
    template <> SpArrayDataType getDataType<uint32_t>() { return SpArrayDataType::UInt32; }
    template <> SpArrayDataType getDataType<int32_t>()  { return SpArrayDataType::Int32; }
    template <> SpArrayDataType getDataType<uint64_t>() { return SpArrayDataType::UInt64; }
    template <> SpArrayDataType getDataType<int64_t>()  { return SpArrayDataType::Int64; }
    template <> SpArrayDataType getDataType<FFloat16>() { return SpArrayDataType::Float16; } // TODO: replace with std::float16_t
    template <> SpArrayDataType getDataType<float>()    { return SpArrayDataType::Float32; }
    template <> SpArrayDataType getDataType<double>()   { return SpArrayDataType::Float64; }

    static int getSizeOf(SpArrayDataType data_type)
    {
        switch (data_type) {
            case SpArrayDataType::UInt8:   return sizeof(uint8_t);
            case SpArrayDataType::Int8:    return sizeof(int8_t);
            case SpArrayDataType::UInt16:  return sizeof(uint16_t);
            case SpArrayDataType::Int16:   return sizeof(int16_t);
            case SpArrayDataType::UInt32:  return sizeof(uint32_t);
            case SpArrayDataType::Int32:   return sizeof(int32_t);
            case SpArrayDataType::UInt64:  return sizeof(uint64_t);
            case SpArrayDataType::Int64:   return sizeof(int64_t);
            case SpArrayDataType::Float16: return sizeof(FFloat16); // TODO: replace with std::float16_t
            case SpArrayDataType::Float32: return sizeof(float);
            case SpArrayDataType::Float64: return sizeof(double);
            default: SP_ASSERT(false); return -1;
        }
    }
};

//
// SpArraySharedMemoryView is similar to a SharedMemoryView, but with an extra member variable that can be
// specified by the system that owns the shared memory to indicate how the shared memory should be used.
//

enum class SpArraySharedMemoryUsageFlags
{
    DoNotUse    = 0,
    Arg         = 1 << 0,
    ReturnValue = 1 << 1
};
SP_DECLARE_ENUM_FLAG_OPERATORS(SpArraySharedMemoryUsageFlags);

// UENUM(Flags) decorator is required to obtain an "A | B | C" string representation from a value
UENUM(Flags)
enum class ESpArraySharedMemoryUsageFlags
{
    DoNotUse    = Unreal::getConstEnumValue(SpArraySharedMemoryUsageFlags::DoNotUse),
    Arg         = Unreal::getConstEnumValue(SpArraySharedMemoryUsageFlags::Arg),
    ReturnValue = Unreal::getConstEnumValue(SpArraySharedMemoryUsageFlags::ReturnValue)
};
ENUM_CLASS_FLAGS(ESpArraySharedMemoryUsageFlags); // required if combining values using bitwise operations

struct SpArraySharedMemoryView : SharedMemoryView
{
    SpArraySharedMemoryView() = default;
    SpArraySharedMemoryView(const SharedMemoryView& view, SpArraySharedMemoryUsageFlags usage_flags) : SharedMemoryView(view) { usage_flags_ = usage_flags; }

    SpArraySharedMemoryUsageFlags usage_flags_ = SpArraySharedMemoryUsageFlags::DoNotUse;
};

//
// SpPackedArray represents an array that can be passed to or returned from an SpFunc. We represent the data
// payload in each SpPackedArray as an std::vector<uint8_t>, regardless of its actual data type, because
// vectors of uint8_t get converted to a more efficient msgpack representation than other types. This results
// in faster data movement. See the following link for details:
//     https://github.com/msgpack/msgpack-c/wiki/v2_0_cpp_adaptor
//

using SpPackedArrayAllocator = SpAlignedAllocator<uint8_t, 4096>;

class SPCORE_API SpPackedArray
{
public:
    inline static constexpr int s_alignment_padding_bytes_ = 4096;

    // typically called before calling an SpFunc to resolve pointers to shared memory
    void resolve();
    void resolve(const SpArraySharedMemoryView& shared_memory_view);

    // typically called before calling an SpFunc to validate that an SpPackedArray can be used as an arg, and
    // after calling an SpFunc to validate that an SpPackedArray can be used as a return value
    void validate(SpArraySharedMemoryUsageFlags usage_flags) const;

    std::vector<uint8_t, SpPackedArrayAllocator> data_;
    void* view_ = nullptr;
    SpArrayDataSource data_source_ = SpArrayDataSource::Invalid;

    std::vector<uint64_t> shape_;
    SpArrayDataType data_type_ = SpArrayDataType::Invalid;

    std::string shared_memory_name_;
    SpArraySharedMemoryUsageFlags shared_memory_usage_flags_ = SpArraySharedMemoryUsageFlags::DoNotUse;
};

//
// SpArray represents a strongly typed array that can be instantiated to manipulate an arg or return value. A
// SpArray object must be converted to an SpPackedArray before it can be passed to, or returned from, an
// SpFunc. To perform this conversion, use the static functions in SpArrayUtils.
//

class SpArrayBase
{
public:
    SpArrayBase() {};                                                               // use when storing in an std::map
    SpArrayBase(const std::string& name) { SP_ASSERT(name != "");  name_ = name; }; // use when storing in an std::vector
    virtual ~SpArrayBase() {};

    // typically called before calling an SpFunc to set args, and from inside an SpFunc to set return values
    virtual SpPackedArray moveToPackedArray() = 0;

    // typically called inside an SpFunc to get args, and after calling an SpFunc to get return values
    virtual void moveFromPackedArray(SpPackedArray&& packed_array) = 0;

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };
    SpArrayBase* getPtr() { return this; };

private:
    std::string name_;
};

template <typename TValue>
class SpArray : public SpArrayBase
{
public:
    SpArray() : SpArrayBase() {};                            // use when storing in an std::map
    SpArray(const std::string& name) : SpArrayBase(name) {}; // use when storing in an std::vector

    // SpArrayBase interface

    // typically called before calling an SpFunc to set args, and from inside an SpFunc to set return values
    SpPackedArray moveToPackedArray() override
    {
        SP_ASSERT(data_source_ != SpArrayDataSource::Invalid);

        SpPackedArray packed_array;

        packed_array.data_ = std::move(data_);
        packed_array.view_ = view_.data();
        packed_array.data_source_ = data_source_;

        packed_array.shape_ = std::move(shape_);
        packed_array.data_type_ = SpArrayDataTypeUtils::getDataType<TValue>();

        packed_array.shared_memory_name_ = std::move(shared_memory_name_);
        packed_array.shared_memory_usage_flags_ = shared_memory_usage_flags_;

        return packed_array;
    }

    // typically called inside an SpFunc to get args, and after calling an SpFunc to get return values
    void moveFromPackedArray(SpPackedArray&& packed_array) override
    {
        SP_ASSERT(packed_array.data_source_ != SpArrayDataSource::Invalid);
        SP_ASSERT(packed_array.data_type_ == SpArrayDataTypeUtils::getDataType<TValue>());

        uint64_t num_elements = 0;
        if (!packed_array.shape_.empty()) {
            num_elements = std::accumulate(packed_array.shape_.begin(), packed_array.shape_.end(), 1, std::multiplies<uint64_t>());
        }

        data_ = std::move(packed_array.data_);
        view_ = std::span<TValue>(static_cast<TValue*>(packed_array.view_), num_elements); // undefined behavior
        data_source_ = packed_array.data_source_;

        shape_ = std::move(packed_array.shape_);

        shared_memory_name_ = std::move(packed_array.shared_memory_name_);
        shared_memory_usage_flags_ = packed_array.shared_memory_usage_flags_;        
    }

    // SpArray interface

    const std::span<TValue>& getView() const { return view_; }
    const std::vector<uint64_t>& getShape() const { return shape_; }

    // setDataSource(...) updates all internal state

    void setDataSource(std::vector<uint8_t, SpPackedArrayAllocator>&& src)
    {
        setDataSource(std::move(src), {-1});
    }

    void setDataSource(std::initializer_list<TValue> initializer_list)
    {
        setDataSource(initializer_list, {-1});
    }

    template <typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, TValue>
    void setDataSource(TRange&& range)
    {
        setDataSource(std::forward<TRange>(range), {-1});
    }

    // when passing in a shape, the shape can include a single -1 as a wildcard, except when passing in a
    // void* or a SharedMemoryView

    void setDataSource(std::vector<uint8_t, SpPackedArrayAllocator>&& src, const std::vector<int64_t>& shape)
    {
        data_ = std::move(src); // src value type is uint8_t, which matches data_, so we don't need to reinterpret
        if (data_.empty()) {
            view_ = std::span<TValue>(static_cast<TValue*>(nullptr), 0);
        } else {
            view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        }
        data_source_ = SpArrayDataSource::Internal;

        shape_ = SpArrayShapeUtils::getShape(shape, view_.size());

        shared_memory_name_ = "";
        shared_memory_usage_flags_ = SpArraySharedMemoryUsageFlags::DoNotUse;
    }

    void setDataSource(std::initializer_list<TValue> initializer_list, const std::vector<int64_t>& shape)
    {
        data_ = Std::reinterpretAsVector<uint8_t, SpPackedArrayAllocator, TValue>(initializer_list);
        if (data_.empty()) {
            view_ = std::span<TValue>(static_cast<TValue*>(nullptr), 0);
        } else {
            view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        }
        data_source_ = SpArrayDataSource::Internal;

        shape_ = SpArrayShapeUtils::getShape(shape, view_.size());

        shared_memory_name_ = "";
        shared_memory_usage_flags_ = SpArraySharedMemoryUsageFlags::DoNotUse;
    }

    template <typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, TValue>
    void setDataSource(TRange&& range, const std::vector<int64_t>& shape)
    {
        data_ = Std::reinterpretAsVector<uint8_t, SpPackedArrayAllocator, TValue>(std::forward<TRange>(range));
        if (data_.empty()) {
            view_ = std::span<TValue>(static_cast<TValue*>(nullptr), 0);
        } else {
            view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        }
        data_source_ = SpArrayDataSource::Internal;

        shape_ = SpArrayShapeUtils::getShape(shape, view_.size());

        shared_memory_name_ = "";
        shared_memory_usage_flags_ = SpArraySharedMemoryUsageFlags::DoNotUse;
    }

    void setDataSource(const void* data, const std::vector<uint64_t>& shape)
    {
        uint64_t num_elements = 0;
        if (!shape.empty()) {
            num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<uint64_t>());
        }

        data_ = {};
        if (num_elements == 0) {
            view_ = std::span<TValue>(static_cast<TValue*>(nullptr), 0);
        } else {
            view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        }
        data_source_ = SpArrayDataSource::External;

        shape_ = shape;

        shared_memory_name_ = "";
        shared_memory_usage_flags_ = SpArraySharedMemoryUsageFlags::DoNotUse;
    }

    void setDataSource(const SpArraySharedMemoryView& shared_memory_view, const std::vector<uint64_t>& shape, const std::string& shared_memory_name)
    {
        uint64_t num_elements = 0;
        if (!shape.empty()) {
            num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<uint64_t>());
        }
        SP_ASSERT(num_elements*sizeof(TValue) <= shared_memory_view.num_bytes_);

        data_ = {};
        if (num_elements == 0) {
            view_ = std::span<TValue>(static_cast<TValue*>(nullptr), 0);
        } else {
            view_ = std::span<TValue>(static_cast<TValue*>(shared_memory_view.data_), num_elements);
        }
        data_source_ = SpArrayDataSource::Shared;

        shape_ = shape;

        shared_memory_name_ = shared_memory_name;
        shared_memory_usage_flags_ = shared_memory_view.usage_flags_;
    }

    // setDataValues(...) sets the underlying data values observed by the packed array, but does not update
    // any internal state

    void setDataValues(std::initializer_list<TValue> src)
    {
        SP_ASSERT(src.size() <= view_.size());
        std::memcpy(view_.data(), std::ranges::data(src), std::ranges::size(src)*sizeof(TValue));
    }

    template <typename TVector> requires
        CVector<TVector> &&
        std::same_as<TValue, typename TVector::value_type>
    void setDataValues(const TVector& src)
    {
        SP_ASSERT(std::ranges::size(src) <= view_.size());
        std::memcpy(view_.data(), src.data(), src.size()*sizeof(TValue));
    }

    template <typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, TValue>
    void setDataValues(TRange&& range)
    {
        TValue* view_ptr = view_.data();
        int i = 0;
        for (auto range_value : range) {
            SP_ASSERT(i < view_.size());
            view_ptr[i] = range_value;
            i++;
        }
    }

private:
    std::vector<uint8_t, SpPackedArrayAllocator> data_;
    std::span<TValue> view_;

    SpArrayDataSource data_source_ = SpArrayDataSource::Invalid;

    std::vector<uint64_t> shape_;

    std::string shared_memory_name_;
    SpArraySharedMemoryUsageFlags shared_memory_usage_flags_ = SpArraySharedMemoryUsageFlags::DoNotUse;
};

//
// SpArrayView is a strongly typed read-only class that can be instantiated to retrieve an arg or return value.
//

class SpArrayViewBase
{
public:
    SpArrayViewBase() {};                                       // use when storing in an std::map
    SpArrayViewBase(const std::string& name) { name_ = name; }; // use when storing in an std::vector
    virtual ~SpArrayViewBase() {};

    // typically called from inside an SpFunc to get arguments, and after calling an SpFunc to get return values
    virtual void setView(const SpPackedArray& packed_array) = 0;

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };
    SpArrayViewBase* getPtr() { return this; };

private:
    std::string name_;
};

template <typename TValue>
class SpArrayView : public SpArrayViewBase
{
public:
    SpArrayView() : SpArrayViewBase("") {};                          // use when storing in an std::map
    SpArrayView(const std::string& name) : SpArrayViewBase(name) {}; // use when storing in an std::vector

    // SpArrayViewBase interface

    // typically called from inside an SpFunc to get arguments, and after calling an SpFunc to get return values
    void setView(const SpPackedArray& packed_array) override
    {
        SP_ASSERT(packed_array.data_type_ == SpArrayDataTypeUtils::getDataType<TValue>());

        uint64_t num_elements = 0;
        if (!packed_array.shape_.empty()) {
            num_elements = std::accumulate(packed_array.shape_.begin(), packed_array.shape_.end(), 1, std::multiplies<uint64_t>());
        }

        view_ = std::span<const TValue>(static_cast<const TValue*>(packed_array.view_), num_elements);
    }

    // SpArrayView interface

    const std::span<const TValue>& getView() const { return view_; }

private:
    std::span<const TValue> view_;
};

//
// SpArrayUtils is a set of helper functions for working with collections of SpArrays and SpPackedArrays.
//

class SPCORE_API SpArrayUtils
{
public:
    SpArrayUtils() = delete;
    ~SpArrayUtils() = delete;

    // typically called at the beginning of an RPC entry point to prepare packed arrays for use in an SpFunc (e.g., by resolving pointers to shared memory)
    static void resolve(std::vector<SpPackedArray>& packed_arrays, const std::map<std::string, SpArraySharedMemoryView>& shared_memory_views);
    static void resolve(std::map<std::string, SpPackedArray>& packed_arrays, const std::map<std::string, SpArraySharedMemoryView>& shared_memory_views);
    static void resolve(SpPackedArray& packed_array, const std::map<std::string, SpArraySharedMemoryView>& shared_memory_views);

    // validate the internal consistency of packed arrays before they are passed to an SpFunc and after they are returned from an SpFunc
    static void validate(const std::vector<SpPackedArray>& packed_arrays, SpArraySharedMemoryUsageFlags usage_flags);
    static void validate(const std::map<std::string, SpPackedArray>& packed_arrays, SpArraySharedMemoryUsageFlags usage_flags);
    static void validate(const SpPackedArray& packed_array, SpArraySharedMemoryUsageFlags usage_flags);

    // typically called before calling an SpFunc to set args, and from inside an SpFunc to set return values
    static std::map<std::string, SpPackedArray> moveToPackedArrays(std::initializer_list<SpArrayBase*> arrays);
    static std::map<std::string, SpPackedArray> moveToPackedArrays(const std::vector<SpArrayBase*>& arrays);
    static std::map<std::string, SpPackedArray> moveToPackedArrays(const std::map<std::string, SpArrayBase*>& arrays);

    // typically called from inside an SpFunc to get args, and after calling an SpFunc to get return values
    static void moveFromPackedArrays(std::initializer_list<SpArrayBase*> arrays, std::map<std::string, SpPackedArray>&& packed_arrays);
    static void moveFromPackedArrays(const std::vector<SpArrayBase*>& arrays, std::map<std::string, SpPackedArray>&& packed_arrays);
    static void moveFromPackedArrays(const std::map<std::string, SpArrayBase*>& arrays, std::map<std::string, SpPackedArray>&& packed_arrays);

    // typically called from inside an SpFunc to get args, and after calling an SpFunc to get return values
    static void setViews(std::initializer_list<SpArrayViewBase*> views, const std::map<std::string, SpPackedArray>& packed_arrays);
    static void setViews(const std::vector<SpArrayViewBase*>& views, const std::map<std::string, SpPackedArray>& packed_arrays);
    static void setViews(const std::map<std::string, SpArrayViewBase*>& views, const std::map<std::string, SpPackedArray>& packed_arrays);
};
