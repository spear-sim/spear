//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t

#include <initializer_list>
#include <map>
#include <span>
#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/Std.h"

//
// The purpose of this file is to assist in defining C++ functions on Unreal components, such that the
// functions can be called from higher-level code dynamically by name. In our naming convention in this
// codebase, we refer to such a function as a "CppFunc". CppFuncs can accept as input, and return as
// output, the following data types: (1) a map of names to data arrays that can each have a different
// data type; (2) a map of names to Unreal objects; and (3) an info string. This file is concerned
// primarily with representing and operating on data arrays.
//

//
// The CppFuncItem class (declared below) is weakly typed, so we use CppFuncDataType to keep track of its
// data type.
//

enum class CppFuncDataType : int8_t
{
    Invalid    = -1,
    UInteger8  = 0,
    Integer8   = 1,
    UInteger16 = 2,
    Integer16  = 3,
    UInteger32 = 4,
    Integer32  = 5,
    UInteger64 = 6,
    Integer64  = 7,
    Float32    = 8,
    Float64    = 9
};

class SPCORE_API CppFuncDataTypeUtils
{
public:
    CppFuncDataTypeUtils() = delete;
    ~CppFuncDataTypeUtils() = delete;

    template <typename TValue> static CppFuncDataType getDataType() { SP_ASSERT(false); return CppFuncDataType::Invalid; }
    template <> CppFuncDataType getDataType<uint8_t>()  { return CppFuncDataType::UInteger8; }
    template <> CppFuncDataType getDataType<int8_t>()   { return CppFuncDataType::Integer8; }
    template <> CppFuncDataType getDataType<uint16_t>() { return CppFuncDataType::UInteger16; }
    template <> CppFuncDataType getDataType<int16_t>()  { return CppFuncDataType::Integer16; }
    template <> CppFuncDataType getDataType<uint32_t>() { return CppFuncDataType::UInteger32; }
    template <> CppFuncDataType getDataType<int32_t>()  { return CppFuncDataType::Integer32; }
    template <> CppFuncDataType getDataType<uint64_t>() { return CppFuncDataType::UInteger64; }
    template <> CppFuncDataType getDataType<int64_t>()  { return CppFuncDataType::Integer64; }
    template <> CppFuncDataType getDataType<float>()    { return CppFuncDataType::Float32; }
    template <> CppFuncDataType getDataType<double>()   { return CppFuncDataType::Float64; }

    static int getSizeOf(CppFuncDataType data_type)
    {
        switch (data_type) {
            case CppFuncDataType::UInteger8:  return sizeof(uint8_t);
            case CppFuncDataType::Integer8:   return sizeof(int8_t);
            case CppFuncDataType::UInteger16: return sizeof(uint16_t);
            case CppFuncDataType::Integer16:  return sizeof(int16_t);
            case CppFuncDataType::UInteger32: return sizeof(uint32_t);
            case CppFuncDataType::Integer32:  return sizeof(int32_t);
            case CppFuncDataType::UInteger64: return sizeof(uint64_t);
            case CppFuncDataType::Integer64:  return sizeof(int64_t);
            case CppFuncDataType::Float32:    return sizeof(float);
            case CppFuncDataType::Float64:    return sizeof(double);
            default: SP_ASSERT(false); return -1;
        }
    }
};

//
// CppFuncSharedMemoryView is similar to a SharedMemoryView, but with an extra member variable that can be
// specified by the system that owns the shared memory to indicate how the shared memory should be used.
//

enum class CppFuncSharedMemoryUsageFlags : uint8_t
{
    DoNotUse    = 0,
    Arg         = 1 << 0,
    ReturnValue = 1 << 1
};

struct CppFuncSharedMemoryView : SharedMemoryView
{
    CppFuncSharedMemoryView(const SharedMemoryView& view, CppFuncSharedMemoryUsageFlags usage_flags)
    {
        id_ = view.id_;
        num_bytes_ = view.num_bytes_;
        data_ = view.data_;
        usage_flags_ = usage_flags;
    }

    CppFuncSharedMemoryUsageFlags usage_flags_;
};

//
// Needed to perform | and & operations on CppFuncSharedMemoryUsageFlags
//

static CppFuncSharedMemoryUsageFlags operator|(CppFuncSharedMemoryUsageFlags lhs, CppFuncSharedMemoryUsageFlags rhs)
{
    return static_cast<CppFuncSharedMemoryUsageFlags>(
        static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(lhs) | static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(rhs));
}

static CppFuncSharedMemoryUsageFlags operator&(CppFuncSharedMemoryUsageFlags lhs, CppFuncSharedMemoryUsageFlags rhs)
{
    return static_cast<CppFuncSharedMemoryUsageFlags>(
        static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(lhs) & static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(rhs));
}

static bool operator||(CppFuncSharedMemoryUsageFlags lhs, bool rhs) // needed for SP_ASSERT
{
    return static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(lhs) || rhs;
}

static bool operator!(CppFuncSharedMemoryUsageFlags val) // needed for SP_ASSERT
{
    return !static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(val);
}

//
// CppFuncItem represents an arg or return value that can be passed to or returned from a CppFunc. We
// represent the data payload in each CppFuncItem as a std::vector<uint8_t>, regardless of its actual data
// type, because vectors of uint8_t get converted to a more efficient MSGPACK representation than other
// types. This results in faster data movement. See the following link for details:
//     https://github.com/msgpack/msgpack-c/wiki/v2_0_cpp_adaptor
//

class CppFuncItem
{
public:
    // Given a CppFuncSharedMemoryView pointer, these resolve methods validate the state of the item, and
    // update it according to whether or not it is intended to be used as an arg or return value. For
    // example, calling resolveArg(...) will update view_ to point to the shared memory, so the item can
    // can be passed to a function, and that function does not need to concern itself with explicitly
    // accessing shared memory.
    void resolveArg(const CppFuncSharedMemoryView* shared_memory_view = nullptr);
    void resolveReturnValue(const CppFuncSharedMemoryView* shared_memory_view = nullptr);

    std::vector<uint8_t> data_;
    void* view_ = nullptr;
    int num_elements_ = -1;
    CppFuncDataType data_type_ = CppFuncDataType::Invalid;
    bool use_shared_memory_ = false;
    std::string shared_memory_name_;
};

//
// CppFuncData is a strongly typed class that can be instantiated to manipulate an arg or return value.
// A CppFuncData object must be converted to a CppFuncItem before it can be passed to, or returned from,
// a CppFunc. To perform this conversion, use the static functions in CppFuncUtils.
//

class CppFuncDataBase
{
public:
    CppFuncDataBase() {};                                                               // use when storing in an std::map
    CppFuncDataBase(const std::string& name) { SP_ASSERT(name != "");  name_ = name; }; // use when storing in an std::vector
    virtual ~CppFuncDataBase() {};

    // typically called before calling a CppFunc to set args, and from inside a CppFunc to set return values
    virtual void moveDataToItem(CppFuncItem& item) = 0;

    // useful for transferring an arg or a return value item to a local data object
    virtual void moveItemToData(CppFuncItem& item) = 0;

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };
    CppFuncDataBase* getPtr() { return this; };

private:
    std::string name_;
};

template <typename TValue>
class CppFuncData : public CppFuncDataBase
{
public:
    CppFuncData() : CppFuncDataBase("") {};                          // use when storing in an std::map
    CppFuncData(const std::string& name) : CppFuncDataBase(name) {}; // use when storing in an std::vector

    // CppFuncDataBase interface
    
    // typically called before calling a CppFunc to set args, and from inside a CppFunc to set return values
    void moveDataToItem(CppFuncItem& item) override
    {
        item.data_ = std::move(data_);
        item.view_ = view_.data();
        item.num_elements_ = view_.size();
        item.data_type_ = CppFuncDataTypeUtils::getDataType<TValue>();
        item.use_shared_memory_ = use_shared_memory_;
        item.shared_memory_name_ = shared_memory_name_;
    }

    // useful for transferring an arg or a return value item to a local data object
    void moveItemToData(CppFuncItem& item)
    {
        SP_ASSERT(item.view_);
        SP_ASSERT(item.num_elements_ >= 0);
        SP_ASSERT(item.data_type_ == CppFuncDataTypeUtils::getDataType<TValue>());
        data_ = std::move(item.data_);
        view_ = std::span<TValue>(static_cast<TValue*>(item.view_), item.num_elements_);
        use_shared_memory_ = item.use_shared_memory_;
        shared_memory_name_ = item.shared_memory_name_;
    }

    // CppFuncData interface

    const std::span<TValue>& getView() { return view_; }

    // setData(...) updates data_ and view_

    void setData(std::vector<uint8_t>&& src)
    {
        data_ = std::move(src); // src value type is uint8_t, which matches data_, so we don't need to reinterpret
        view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        use_shared_memory_ = false;
        shared_memory_name_ = "";
    }

    void setData(std::initializer_list<TValue> initializer_list)
    {
        data_ = Std::reinterpretAsVector<uint8_t, TValue>(initializer_list);
        view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        use_shared_memory_ = false;
        shared_memory_name_ = "";
    }

    template <typename TRange> requires CRangeHasValuesConvertibleTo<TRange, TValue>
    void setData(TRange&& range)
    {
        data_ = Std::reinterpretAsVector<uint8_t, TValue>(std::forward<decltype(range)>(range));
        view_ = Std::reinterpretAsSpanOf<TValue>(data_);
        use_shared_memory_ = false;
        shared_memory_name_ = "";
    }

    void setData(const std::string& shared_memory_name, const SharedMemoryView& shared_memory_view, int num_elements)
    {
        SP_ASSERT(num_elements*sizeof(TValue) <= shared_memory_view.num_bytes_);
        data_ = {};
        view_ = std::span<TValue>(static_cast<TValue*>(shared_memory_view.data_), num_elements);
        use_shared_memory_ = true;
        shared_memory_name_ = shared_memory_name;
    }

    // setValues(...) sets the underlying values observed by view_ without updating data_ or view_

    void setValues(std::initializer_list<TValue> src)
    {
        SP_ASSERT(std::ranges::size(src) <= view_.size());
        std::memcpy(view_.data(), std::ranges::data(src), std::ranges::size(src)*sizeof(TValue));
    }

    template <typename TSpan> requires CSpan<TSpan> && std::same_as<typename TSpan::value_type, TValue>
    void setValues(const TSpan& src)
    {
        SP_ASSERT(src.size() <= view_.size());
        std::memcpy(view_.data(), src.data(), src.size()*sizeof(TValue));
    }

    template <typename TRange> requires CRangeHasValuesConvertibleTo<TRange, TValue>
    void setValues(TRange&& range)
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
    std::vector<uint8_t> data_;
    std::span<TValue> view_;

    bool use_shared_memory_ = false;
    std::string shared_memory_name_;
};

//
// CppFuncView is a {strongly typed, read-only} class that can be instantiated to retrieve an arg or return value.
//

class CppFuncViewBase
{
public:
    CppFuncViewBase() {};                                       // use when storing in an std::map
    CppFuncViewBase(const std::string& name) { name_ = name; }; // use when storing in an std::vector
    virtual ~CppFuncViewBase() {};

    // typically called from inside a CppFunc to retrieve args, and after calling a CppFunc to retrieve return values
    virtual void setViewFromItem(const CppFuncItem& item) = 0;

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };
    CppFuncViewBase* getPtr() { return this; };

private:
    std::string name_;
};

template <typename TValue>
class CppFuncView : public CppFuncViewBase
{
public:
    CppFuncView() : CppFuncViewBase("") {};                          // use when storing in an std::map
    CppFuncView(const std::string& name) : CppFuncViewBase(name) {}; // use when storing in an std::vector

    // CppFuncViewBase interface, typically called from inside a CppFunc to retrieve args, and after calling a CppFunc to retrieve return values
    void setViewFromItem(const CppFuncItem& item) override
    {
        SP_ASSERT(item.data_type_ == CppFuncDataTypeUtils::getDataType<TValue>());
        view_ = std::span<const TValue>(static_cast<const TValue*>(item.view_), item.num_elements_);
    }

    // CppFuncView interface
    const std::span<const TValue>& getView() { return view_; }

private:
    std::span<const TValue> view_;
};

//
// CppFuncPackage is intended as a high-level helper struct that can be used as a one-size-fits-all arg and
// return type for CppFuncs. We choose to make this a struct so it will be easier to add fields if necessary,
// without needing to explicitly update the signature of every CppFunc.
//

struct CppFuncPackage
{
    std::map<std::string, CppFuncItem> items_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};

//
// CppFuncUtils is a set of static helper functions for manipulating collections fo items and data.
//

class SPCORE_API CppFuncUtils
{
public:
    CppFuncUtils() = delete;
    ~CppFuncUtils() = delete;

    // typically called at the beginning of a C++ entry point to prepare items for use (e.g., by resolving pointers to shared memory)
    static void resolveArg(CppFuncItem& arg, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views);
    static void resolveArgs(std::vector<CppFuncItem>& args, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views);
    static void resolveArgs(std::map<std::string, CppFuncItem>& args, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views);

    // typically called at the end of a C++ entry point to prepare items for being returned (e.g., by validating the internal consistency of each item)
    static void resolveReturnValue(CppFuncItem& return_value, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views);
    static void resolveReturnValues(std::vector<CppFuncItem>& return_values, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views);
    static void resolveReturnValues(std::map<std::string, CppFuncItem>& return_values, const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views);

    // typically called before calling a CppFunc to set args, and from inside a CppFunc to set return values
    static std::map<std::string, CppFuncItem> moveDataToItems(std::initializer_list<CppFuncDataBase*> data_objs);
    static std::map<std::string, CppFuncItem> moveDataToItems(const std::vector<CppFuncDataBase*>& data_objs);
    static std::map<std::string, CppFuncItem> moveDataToItems(const std::map<std::string, CppFuncDataBase*>& data_objs);

    // typically called when transferring an arg or a return value item to a local data object
    static void moveItemsToData(std::initializer_list<CppFuncDataBase*> data_objs, std::map<std::string, CppFuncItem>& items);
    static void moveItemsToData(const std::vector<CppFuncDataBase*>& data_objs, std::map<std::string, CppFuncItem>& items);
    static void moveItemsToData(const std::map<std::string, CppFuncDataBase*>& data_objs, std::map<std::string, CppFuncItem>& items);

    // typically called from inside a CppFunc to retrieve args, and after calling a CppFunc to retrieve return values
    static void setViewsFromItems(std::initializer_list<CppFuncViewBase*> view_objs, const std::map<std::string, CppFuncItem>& items);
    static void setViewsFromItems(const std::vector<CppFuncViewBase*>& view_objs, const std::map<std::string, CppFuncItem>& items);
    static void setViewsFromItems(const std::map<std::string, CppFuncViewBase*>& view_objs, const std::map<std::string, CppFuncItem>& items);
};
