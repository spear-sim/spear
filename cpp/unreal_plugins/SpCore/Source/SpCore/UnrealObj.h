//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <initializer_list>
#include <map>
#include <string>
#include <vector>

#include <HAL/Platform.h> // SPCORE_API

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistrar.h"

class UStruct;

class UnrealObjBase
{
public:
    UnrealObjBase() {};                                       // use when storing in an std::map
    UnrealObjBase(const std::string& name) { name_ = name; }; // use when storing in an std::vector
    virtual ~UnrealObjBase() {};

    virtual void* getValuePtr() = 0;
    virtual UStruct* getStaticStruct() const = 0;

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };
    UnrealObjBase* getPtr() { return this; }

private:
    std::string name_;
};

template <typename TObj>
class UnrealObj : public UnrealObjBase
{
public:
    UnrealObj() {};                                              // use when storing in an std::map
    UnrealObj(const std::string& name) : UnrealObjBase(name) {}; // use when storing in an std::vector

    void* getValuePtr() override { return &obj_; }
    UStruct* getStaticStruct() const override { return UnrealClassRegistrar::getStaticStruct<TObj>(); }

    const TObj& getObj() const { return obj_; }
    void setObj(const TObj& obj) { obj_ = obj; }

private:
    TObj obj_;
};

class SPCORE_API UnrealObjUtils
{
public:
    UnrealObjUtils() = delete;
    ~UnrealObjUtils() = delete;

    static std::map<std::string, std::string> getObjectPropertiesAsStrings(std::initializer_list<UnrealObjBase*> unreal_objs);
    static std::map<std::string, std::string> getObjectPropertiesAsStrings(const std::vector<UnrealObjBase*>& unreal_objs);
    static std::map<std::string, std::string> getObjectPropertiesAsStrings(const std::map<std::string, UnrealObjBase*>& unreal_objs);

    static void setObjectPropertiesFromStrings(std::initializer_list<UnrealObjBase*> unreal_objs, const std::map<std::string, std::string>& strings);
    static void setObjectPropertiesFromStrings(const std::vector<UnrealObjBase*>& unreal_objs, const std::map<std::string, std::string>& strings);
    static void setObjectPropertiesFromStrings(const std::map<std::string, UnrealObjBase*>& unreal_objs, const std::map<std::string, std::string>& strings);
};
