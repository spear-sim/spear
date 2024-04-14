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
#include "SpCore/Std.h"

// Needs to match SpEngine/CppFuncService.h
enum class CppFuncDataType
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

struct CppFuncArg
{
    std::span<const uint8_t> data_;
    CppFuncDataType data_type_ = CppFuncDataType::Invalid;
};

struct CppFuncReturnValue
{
    std::vector<uint8_t> data_;
    CppFuncDataType data_type_ = CppFuncDataType::Invalid;
    bool use_shared_memory_ = false;
    int shared_memory_num_bytes_ = -1;
    std::string shared_memory_name_;
};

class CppFuncDataBase
{
public:
    CppFuncDataBase() {};                                       // use when storing in an std::map
    CppFuncDataBase(const std::string& name) { name_ = name; }; // use when storing in an std::vector
    virtual ~CppFuncDataBase() {};

    virtual void updateArg(CppFuncArg& arg) const = 0;                        // typically called before calling a function to set args
    virtual void setData(const CppFuncArg& arg) = 0;                          // typically called from inside a function to retrieve args
    virtual void moveDataToReturnValue(CppFuncReturnValue& return_value) = 0; // typically called from inside a function to set return values
    virtual void setData(const CppFuncReturnValue& return_value) = 0;         // typically called after calling a function to retrieve return values

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };
    CppFuncDataBase* getPtr() { return this; };

private:
    std::string name_;
};

template <typename TValue>
class CppFuncData : public CppFuncDataBase
{
public:
    CppFuncData() {};                                                // use when storing in an std::map
    CppFuncData(const std::string& name) : CppFuncDataBase(name) {}; // use when storing in an std::vector

    //
    // CppFuncDataBase interface
    //

    // typically called before calling a function to set args
    void updateArg(CppFuncArg& arg) const override
    {
        arg.data_ = Std::reinterpretAsSpanOf<const uint8_t>(view_);
        arg.data_type_ = getDataType<TValue>();
    }

    // typically called from inside a function to retrieve args
    void setData(const CppFuncArg& arg) override
    {
        SP_ASSERT(arg.data_type_ == getDataType<TValue>());
        data_ = {};
        view_ = Std::reinterpretAsSpanOf<const TValue>(arg.data_);
    }

    // typically called from inside a function to set return values
    void moveDataToReturnValue(CppFuncReturnValue& return_value) override
    {
        return_value.data_ = std::move(data_);
        return_value.data_type_ = getDataType<TValue>();
        data_ = {};
        view_ = std::span<const TValue>();
    }

    // typically called after returning from a function to retrieve return values
    void setData(const CppFuncReturnValue& return_value) override
    {
        SP_ASSERT(return_value.data_type_ == getDataType<TValue>());
        data_ = {};
        view_ = Std::reinterpretAsSpanOf<const TValue>(return_value.data_);
    }

    //
    // CppFuncData interface
    //

    std::span<const TValue>& getData() { return view_; }

    void setData(std::vector<uint8_t>&& src)
    {
        data_ = std::move(src); // can be moved because no reinterpreting is required when assigning to data_
        view_ = Std::reinterpretAsSpanOf<const TValue>(data_);
    }

    void setData(std::initializer_list<TValue> initializer_list)
    {
        data_ = Std::reinterpretAsVector<uint8_t, TValue>(initializer_list);
        view_ = Std::reinterpretAsSpanOf<const TValue>(data_);
    }

    template <typename TRange> requires CRangeHasValuesConvertibleTo<TRange, TValue>
    void setData(TRange&& range)
    {
        data_ = Std::reinterpretAsVector<uint8_t, TValue>(std::forward<decltype(range)>(range));
        view_ = Std::reinterpretAsSpanOf<const TValue>(data_);
    }

private:
    template <typename TValue> static CppFuncDataType getDataType() { SP_ASSERT(false); return CppFuncDataType::Invalid; }
    template <> CppFuncDataType getDataType<uint8_t>() { return CppFuncDataType::UInteger8; }
    template <> CppFuncDataType getDataType<int8_t>() { return CppFuncDataType::Integer8; }
    template <> CppFuncDataType getDataType<uint16_t>() { return CppFuncDataType::UInteger16; }
    template <> CppFuncDataType getDataType<int16_t>() { return CppFuncDataType::Integer16; }
    template <> CppFuncDataType getDataType<uint32_t>() { return CppFuncDataType::UInteger32; }
    template <> CppFuncDataType getDataType<int32_t>() { return CppFuncDataType::Integer32; }
    template <> CppFuncDataType getDataType<uint64_t>() { return CppFuncDataType::UInteger64; }
    template <> CppFuncDataType getDataType<int64_t>() { return CppFuncDataType::Integer64; }
    template <> CppFuncDataType getDataType<float>() { return CppFuncDataType::Float32; }
    template <> CppFuncDataType getDataType<double>() { return CppFuncDataType::Float64; }

    std::vector<uint8_t> data_;
    std::span<const TValue> view_;
};

class SPCORE_API CppFuncDataUtils
{
public:
    CppFuncDataUtils() = delete;
    ~CppFuncDataUtils() = delete;

    // typically called before calling a function to set args
    static std::map<std::string, CppFuncArg> getArgsFromData(std::initializer_list<CppFuncDataBase*> data_objs);
    static std::map<std::string, CppFuncArg> getArgsFromData(const std::vector<CppFuncDataBase*>& data_objs);
    static std::map<std::string, CppFuncArg> getArgsFromData(const std::map<std::string, CppFuncDataBase*>& data_objs);

    // typically called from inside a function to retrieve args
    static void setDataFromArgs(std::initializer_list<CppFuncDataBase*> data_objs, const std::map<std::string, CppFuncArg>& args);
    static void setDataFromArgs(const std::vector<CppFuncDataBase*>& data_objs, const std::map<std::string, CppFuncArg>& args);
    static void setDataFromArgs(const std::map<std::string, CppFuncDataBase*>& data_objs, const std::map<std::string, CppFuncArg>& args);

    // typically called from inside a function to set return values
    static std::map<std::string, CppFuncReturnValue> getReturnValuesFromData(std::initializer_list<CppFuncDataBase*> data_objs);
    static std::map<std::string, CppFuncReturnValue> getReturnValuesFromData(const std::vector<CppFuncDataBase*>& data_objs);
    static std::map<std::string, CppFuncReturnValue> getReturnValuesFromData(const std::map<std::string, CppFuncDataBase*>& data_objs);

    // typically called after returning from a function to retrieve return values
    static void setDataFromReturnValues(std::initializer_list<CppFuncDataBase*> data_objs, const std::map<std::string, CppFuncReturnValue>& return_values);
    static void setDataFromReturnValues(const std::vector<CppFuncDataBase*>& data_objs, const std::map<std::string, CppFuncReturnValue>& return_values);
    static void setDataFromReturnValues(const std::map<std::string, CppFuncDataBase*>& data_objs, const std::map<std::string, CppFuncReturnValue>& return_values);
};
