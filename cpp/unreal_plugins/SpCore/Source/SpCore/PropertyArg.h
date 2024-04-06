//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>    // std::derived_from, std::same_as
#include <initializer_list>
#include <map>
#include <string>
#include <utility>     // std::make_pair
#include <type_traits> // std::remove_pointer_t
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistrar.h"

class UStruct;

class PropertyArgBase
{
public:
    PropertyArgBase() {};                                       // use when storing in an std::map
    PropertyArgBase(const std::string& name) { name_ = name; }; // use when storing in an std::vector
    virtual ~PropertyArgBase() {};

    virtual void* getValuePtr() = 0;
    virtual UStruct* getStaticStruct() const = 0;

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };

private:
    std::string name_;
};

template <typename T>
class PropertyArg : public PropertyArgBase
{
public:
    PropertyArg() {};                                                // use when storing in an std::map
    PropertyArg(const std::string& name) : PropertyArgBase(name) {}; // use when storing in an std::vector

    void* getValuePtr() override { return &value; }
    UStruct* getStaticStruct() const override { return UnrealClassRegistrar::getStaticStruct<T>(); }

    T& get() const { return value; }

private:
    T value;
};

class PropertyArgUtils
{
public:
    PropertyArgUtils() = delete;
    ~PropertyArgUtils() = delete;

    template <typename TValue> requires
        std::derived_from<std::remove_pointer_t<TValue>, PropertyArgBase>
    static void setArgs(std::initializer_list<TValue> property_args, const std::map<std::string, std::string>& property_strings)
    {
        std::vector<TValue> property_args_vector = Std::toVector<TValue>(property_args);
        setArgs(property_args_vector, property_strings);
    }

    template <CValueContainer TValueContainer> requires
        std::derived_from<std::remove_pointer_t<typename TValueContainer::value_type>, PropertyArgBase>
    static void setArgs(const TValueContainer& property_args, const std::map<std::string, std::string>& property_strings)
    {
        SP_ASSERT(!Std::contains(property_args, nullptr));
        setArgs(
            Std::toMap<std::string, PropertyArgBase*>(
                property_args | std::views::transform([](auto property_arg) { return std::make_pair(property_arg->getName(), property_arg); })),
            property_strings);
    }

    template <CKeyValueContainer TKeyValueContainer> requires
        std::same_as<typename TKeyValueContainer::key_type, std::string> &&
        std::derived_from<std::remove_pointer_t<typename TKeyValueContainer::mapped_type>, PropertyArgBase>
    static void setArgs(const TKeyValueContainer& property_args, const std::map<std::string, std::string>& property_strings)
    {
        for (auto& [property_name, property_string] : property_strings) {
            SP_ASSERT(Std::containsKey(property_args, property_name));
            PropertyArgBase* property_arg = property_args.at(property_name);
            SP_ASSERT(property_arg);
            Unreal::setObjectPropertiesFromString(property_arg->getValuePtr(), property_arg->getStaticStruct(), property_string);
        }
    }
};
