//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t

#include <initializer_list>
#include <functional> // std::multiplies
#include <map>
#include <numeric>    // std::accumulate
#include <ranges>     // std::ranges::data, std::views::filter, std::ranges::size
#include <span>
#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/Std.h"

//
// each SpFuncArray has a data type (uint8, float32, etc) and a data source (backed by an internal
// std::vector, backed by an external pointer, backed by shared memory, etc)
//

enum class SpFuncArrayDataType : int8_t
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
    Float32 = 8,
    Float64 = 9
};

enum class SpFuncArrayDataSource : int8_t
{
    Invalid  = -1,
    Internal = 0,
    External = 1,
    Shared   = 2
};

class SPCORE_API SpFuncArrayDataTypeUtils
{
public:
    SpFuncArrayDataTypeUtils() = delete;
    ~SpFuncArrayDataTypeUtils() = delete;

    template <typename TValue> static SpFuncArrayDataType getDataType() { SP_ASSERT(false); return SpFuncArrayDataType::Invalid; }
    template <> SpFuncArrayDataType getDataType<uint8_t>()  { return SpFuncArrayDataType::UInt8; }
    template <> SpFuncArrayDataType getDataType<int8_t>()   { return SpFuncArrayDataType::Int8; }
    template <> SpFuncArrayDataType getDataType<uint16_t>() { return SpFuncArrayDataType::UInt16; }
    template <> SpFuncArrayDataType getDataType<int16_t>()  { return SpFuncArrayDataType::Int16; }
    template <> SpFuncArrayDataType getDataType<uint32_t>() { return SpFuncArrayDataType::UInt32; }
    template <> SpFuncArrayDataType getDataType<int32_t>()  { return SpFuncArrayDataType::Int32; }
    template <> SpFuncArrayDataType getDataType<uint64_t>() { return SpFuncArrayDataType::UInt64; }
    template <> SpFuncArrayDataType getDataType<int64_t>()  { return SpFuncArrayDataType::Int64; }
    template <> SpFuncArrayDataType getDataType<float>()    { return SpFuncArrayDataType::Float32; }
    template <> SpFuncArrayDataType getDataType<double>()   { return SpFuncArrayDataType::Float64; }

    static int getSizeOf(SpFuncArrayDataType data_type)
    {
        switch (data_type) {
            case SpFuncArrayDataType::UInt8:   return sizeof(uint8_t);
            case SpFuncArrayDataType::Int8:    return sizeof(int8_t);
            case SpFuncArrayDataType::UInt16:  return sizeof(uint16_t);
            case SpFuncArrayDataType::Int16:   return sizeof(int16_t);
            case SpFuncArrayDataType::UInt32:  return sizeof(uint32_t);
            case SpFuncArrayDataType::Int32:   return sizeof(int32_t);
            case SpFuncArrayDataType::UInt64:  return sizeof(uint64_t);
            case SpFuncArrayDataType::Int64:   return sizeof(int64_t);
            case SpFuncArrayDataType::Float32: return sizeof(float);
            case SpFuncArrayDataType::Float64: return sizeof(double);
            default: SP_ASSERT(false); return -1;
        }
    }
};

//
// SpFuncSharedMemoryView is similar to a SharedMemoryView, but with an extra member variable that can be
// specified by the system that owns the shared memory to indicate how the shared memory should be used.
//

enum class SpFuncSharedMemoryUsageFlags : uint8_t
{
    DoNotUse    = 0,
    Arg         = 1 << 0,
    ReturnValue = 1 << 1
};
SP_DECLARE_ENUM_FLAG_OPERATORS(SpFuncSharedMemoryUsageFlags);

struct SpFuncSharedMemoryView : SharedMemoryView
{
    SpFuncSharedMemoryView() = default;
    SpFuncSharedMemoryView(const SharedMemoryView& view, SpFuncSharedMemoryUsageFlags usage_flags) : SharedMemoryView(view)
    {
        usage_flags_ = usage_flags;
    }

    SpFuncSharedMemoryUsageFlags usage_flags_ = SpFuncSharedMemoryUsageFlags::DoNotUse;
};

//
// SpFuncPackedArray represents an array that can be passed to or returned from an SpFunc. We represent the
// data payload in each SpFuncPackedArray as an std::vector<uint8_t>, regardless of its actual data type,
// because vectors of uint8_t get converted to a more efficient MSGPACK representation than other types. This
// results in faster data movement. See the following link for details:
//     https://github.com/msgpack/msgpack-c/wiki/v2_0_cpp_adaptor
//

class SpFuncPackedArray
{
public:
    void setView(const SpFuncSharedMemoryView& shared_memory_view);
    void validate(SpFuncSharedMemoryUsageFlags usage_flags) const;

    std::vector<uint8_t> data_;
    void* view_ = nullptr;
    SpFuncArrayDataSource data_source_ = SpFuncArrayDataSource::Invalid;

    std::vector<uint64_t> shape_;
    SpFuncArrayDataType data_type_ = SpFuncArrayDataType::Invalid;

    std::string shared_memory_name_;
    SpFuncSharedMemoryUsageFlags shared_memory_usage_flags_ = SpFuncSharedMemoryUsageFlags::DoNotUse;
};

//
// SpFuncArray represents a strongly typed array that can be instantiated to manipulate an arg or return value.
// A SpFuncArray object must be converted to an SpFuncPackedArray before it can be passed to, or returned from,
// an SpFunc. To perform this conversion, use the static functions in SpFuncArrayUtils.
//

class SpFuncArrayBase
{
public:
    SpFuncArrayBase() {};                                                               // use when storing in an std::map
    SpFuncArrayBase(const std::string& name) { SP_ASSERT(name != "");  name_ = name; }; // use when storing in an std::vector
    virtual ~SpFuncArrayBase() {};

    // typically called before calling an SpFunc to set args, and from inside an SpFunc to set return values
    virtual void moveToPackedArray(SpFuncPackedArray& packed_array) = 0;

    // typically called inside an SpFunc to get args, and after calling an SpFunc to get return values
    virtual void moveFromPackedArray(SpFuncPackedArray& packed_array) = 0;

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };
    SpFuncArrayBase* getPtr() { return this; };

private:
    std::string name_;
};

template <typename TValue>
class SpFuncArray : public SpFuncArrayBase
{
public:
    SpFuncArray() : SpFuncArrayBase("") {};                          // use when storing in an std::map
    SpFuncArray(const std::string& name) : SpFuncArrayBase(name) {}; // use when storing in an std::vector

    // SpFuncArrayBase interface

    // typically called before calling an SpFunc to set args, and from inside an SpFunc to set return values
    void moveToPackedArray(SpFuncPackedArray& packed_array) override
    {
        packed_array.data_ = std::move(data_);
        packed_array.view_ = view_.data();
        packed_array.data_source_ = data_source_;

        packed_array.shape_ = std::move(shape_);
        packed_array.data_type_ = SpFuncArrayDataTypeUtils::getDataType<TValue>();

        packed_array.shared_memory_name_ = std::move(shared_memory_name_);
        packed_array.shared_memory_usage_flags_ = shared_memory_usage_flags_;
    }

    // typically called from inside an SpFunc to get args, and after calling an SpFunc to get return values
    void moveFromPackedArray(SpFuncPackedArray& packed_array)
    {
        SP_ASSERT(packed_array.data_source_ != SpFuncArrayDataSource::Invalid);
        SP_ASSERT(packed_array.data_type_ == SpFuncArrayDataTypeUtils::getDataType<TValue>());

        uint64_t num_elements = 0;
        if (!packed_array.shape_.empty()) {
            num_elements = std::accumulate(packed_array.shape_.begin(), packed_array.shape_.end(), 1, std::multiplies<uint64_t>());
        }

        data_ = std::move(packed_array.data_);
        view_ = std::span<TValue>(static_cast<TValue*>(packed_array.view_), num_elements);
        data_source_ = packed_array.data_source_;

        shape_ = std::move(packed_array.shape_);

        shared_memory_name_ = std::move(packed_array.shared_memory_name_);
        shared_memory_usage_flags_ = packed_array.shared_memory_usage_flags_;
    }

    // SpFuncArray interface

    const std::span<TValue>& getView() { return view_; }

    // setData(...) updates all internal state

    void setData(std::vector<uint8_t>&& src)
    {
        setData(std::move(src), {-1});
    }

    void setData(std::initializer_list<TValue> initializer_list)
    {
        setData(initializer_list, {-1});
    }

    template <typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, TValue>
    void setData(TRange&& range)
    {
        setData(std::forward<decltype(range)>(range), {-1});
    }

    // setData(...) updates all internal state, when passing in a shape, the shape can include a single -1 as
    // a wildcard, except when passing in a void* or a SharedMemoryView

    void setData(std::vector<uint8_t>&& src, const std::vector<int64_t>& shape)
    {
        data_ = std::move(src); // src value type is uint8_t, which matches data_, so we don't need to reinterpret
        view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        data_source_ = SpFuncArrayDataSource::Internal;

        shape_ = getShape(shape, view_.size());

        shared_memory_name_ = "";
        shared_memory_usage_flags_ = SpFuncSharedMemoryUsageFlags::DoNotUse;
    }

    void setData(std::initializer_list<TValue> initializer_list, const std::vector<int64_t>& shape)
    {
        data_ = Std::reinterpretAsVector<uint8_t, TValue>(initializer_list);
        view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        data_source_ = SpFuncArrayDataSource::Internal;

        shape_ = getShape(shape, view_.size());

        shared_memory_name_ = "";
        shared_memory_usage_flags_ = SpFuncSharedMemoryUsageFlags::DoNotUse;
    }

    template <typename TRange> requires
        CRangeValuesAreConvertibleTo<TRange, TValue>
    void setData(TRange&& range, const std::vector<int64_t>& shape)
    {
        data_ = Std::reinterpretAsVector<uint8_t, TValue>(std::forward<decltype(range)>(range));
        view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        data_source_ = SpFuncArrayDataSource::Internal;

        shape_ = getShape(shape, view_.size());

        shared_memory_name_ = "";
        shared_memory_usage_flags_ = SpFuncSharedMemoryUsageFlags::DoNotUse;
    }

    void setData(const void* data, const std::vector<uint64_t>& shape)
    {
        uint64_t num_elements = 0;
        if (!shape.empty()) {
            num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<uint64_t>());
        }

        data_ = {};
        view_ = std::span<TValue>(static_cast<TValue*>(data), num_elements);
        data_source_ = SpFuncArrayDataSource::External;

        shape_ = shape;

        shared_memory_name_ = "";
        shared_memory_usage_flags_ = SpFuncSharedMemoryUsageFlags::DoNotUse;
    }

    void setData(const std::string& shared_memory_name, const SpFuncSharedMemoryView& shared_memory_view, const std::vector<uint64_t>& shape)
    {
        uint64_t num_elements = 0;
        if (!shape.empty()) {
            num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<uint64_t>());
        }
        SP_ASSERT(num_elements*sizeof(TValue) <= shared_memory_view.num_bytes_);

        data_ = {};
        view_ = std::span<TValue>(static_cast<TValue*>(shared_memory_view.data_), num_elements);
        data_source_ = SpFuncArrayDataSource::Shared;

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
    static std::vector<uint64_t> getShape(const std::vector<int64_t>& shape, uint64_t num_elements)
    {
        SP_ASSERT(Std::toVector<uint64_t>(shape | std::views::filter([](auto s) { return s < -1; })).size() == 0); // must contain no entries less than -1
        SP_ASSERT(Std::toVector<uint64_t>(shape | std::views::filter([](auto s) { return s == -1; })).size() <= 1); // must contain at most one -1

        // If shape is {}, then 0 is the only value for num_elements that matches shape, and return {}.
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

    std::vector<uint8_t> data_;
    std::span<TValue> view_;
    SpFuncArrayDataSource data_source_ = SpFuncArrayDataSource::Invalid;

    std::vector<uint64_t> shape_;

    std::string shared_memory_name_;
    SpFuncSharedMemoryUsageFlags shared_memory_usage_flags_ = SpFuncSharedMemoryUsageFlags::DoNotUse;
};

//
// SpFuncArrayView is a strongly typed read-only class that can be instantiated to retrieve an arg or return value.
//

class SpFuncArrayViewBase
{
public:
    SpFuncArrayViewBase() {};                                       // use when storing in an std::map
    SpFuncArrayViewBase(const std::string& name) { name_ = name; }; // use when storing in an std::vector
    virtual ~SpFuncArrayViewBase() {};

    // typically called from inside an SpFunc to get arguments, and after calling an SpFunc to get return values
    virtual void setView(const SpFuncPackedArray& packed_array) = 0;

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };
    SpFuncArrayViewBase* getPtr() { return this; };

private:
    std::string name_;
};

template <typename TValue>
class SpFuncArrayView : public SpFuncArrayViewBase
{
public:
    SpFuncArrayView() : SpFuncArrayViewBase("") {};                          // use when storing in an std::map
    SpFuncArrayView(const std::string& name) : SpFuncArrayViewBase(name) {}; // use when storing in an std::vector

    // SpFuncArrayViewBase interface

    // typically called from inside an SpFunc to get arguments, and after calling an SpFunc to get return values
    void setView(const SpFuncPackedArray& packed_array) override
    {
        SP_ASSERT(packed_array.data_type_ == SpFuncArrayDataTypeUtils::getDataType<TValue>());

        uint64_t num_elements = 0;
        if (!packed_array.shape_.empty()) {
            num_elements = std::accumulate(packed_array.shape_.begin(), packed_array.shape_.end(), 1, std::multiplies<uint64_t>());
        }

        view_ = std::span<const TValue>(static_cast<const TValue*>(packed_array.view_), num_elements);
    }

    // SpFuncArrayView interface

    const std::span<const TValue>& getView() { return view_; }

private:
    std::span<const TValue> view_;
};

//
// SpFuncArrayUtils is a set of helper functions for working with collections of SpFuncArrays and
// SpFuncPackedArrays.
//

class SPCORE_API SpFuncArrayUtils
{
public:
    SpFuncArrayUtils() = delete;
    ~SpFuncArrayUtils() = delete;

    // typically called at the beginning of an RPC entry point to prepare packed arrays for use in an SpFunc (e.g., by resolving pointers to shared memory)
    static void resolve(SpFuncPackedArray& packed_array, const std::map<std::string, SpFuncSharedMemoryView>& shared_memory_views);
    static void resolve(std::vector<SpFuncPackedArray>& packed_arrays, const std::map<std::string, SpFuncSharedMemoryView>& shared_memory_views);
    static void resolve(std::map<std::string, SpFuncPackedArray>& packed_arrays, const std::map<std::string, SpFuncSharedMemoryView>& shared_memory_views);

    // validate the internal consistency of packed arrays before they are passed to an SpFunc and after they are returned from an SpFunc
    static void validate(const SpFuncPackedArray& packed_array, SpFuncSharedMemoryUsageFlags usage_flags = SpFuncSharedMemoryUsageFlags::DoNotUse);
    static void validate(const std::vector<SpFuncPackedArray>& packed_arrays, SpFuncSharedMemoryUsageFlags usage_flags = SpFuncSharedMemoryUsageFlags::DoNotUse);
    static void validate(const std::map<std::string, SpFuncPackedArray>& packed_arrays, SpFuncSharedMemoryUsageFlags usage_flags = SpFuncSharedMemoryUsageFlags::DoNotUse);

    // typically called before calling an SpFunc to set args, and from inside an SpFunc to set return values
    static std::map<std::string, SpFuncPackedArray> moveToPackedArrays(std::initializer_list<SpFuncArrayBase*> arrays);
    static std::map<std::string, SpFuncPackedArray> moveToPackedArrays(const std::vector<SpFuncArrayBase*>& arrays);
    static std::map<std::string, SpFuncPackedArray> moveToPackedArrays(const std::map<std::string, SpFuncArrayBase*>& arrays);

    // typically called from inside an SpFunc to get args, and after calling an SpFunc to get return values
    static void moveFromPackedArrays(std::initializer_list<SpFuncArrayBase*> arrays, std::map<std::string, SpFuncPackedArray>& packed_arrays);
    static void moveFromPackedArrays(const std::vector<SpFuncArrayBase*>& arrays, std::map<std::string, SpFuncPackedArray>& packed_arrays);
    static void moveFromPackedArrays(const std::map<std::string, SpFuncArrayBase*>& arrays, std::map<std::string, SpFuncPackedArray>& packed_arrays);

    // typically called from inside an SpFunc to get args, and after calling an SpFunc to get return values
    static void setViewsFromPackedArrays(std::initializer_list<SpFuncArrayViewBase*> arrays, const std::map<std::string, SpFuncPackedArray>& packed_arrays);
    static void setViewsFromPackedArrays(const std::vector<SpFuncArrayViewBase*>& arrays, const std::map<std::string, SpFuncPackedArray>& packed_arrays);
    static void setViewsFromPackedArrays(const std::map<std::string, SpFuncArrayViewBase*>& arrays, const std::map<std::string, SpFuncPackedArray>& packed_arrays);
};

//
// SpFuncDataBundle is intended as a high-level helper struct that can be used as the argument to, and the
// return value from, an SpFunc. We choose to make this a struct so it will be easier to add fields if
// necessary, without needing to explicitly update the signature of every SpFunc.
//

struct SpFuncDataBundle
{
    std::map<std::string, SpFuncPackedArray> packed_arrays_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};
