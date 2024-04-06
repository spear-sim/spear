//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>    // std::derived_from, std::same_as
#include <initializer_list>
#include <map>
#include <string>
#include <utility>     // std::make_pair
#include <type_traits> // std::is_pointer_t, std::remove_pointer_t
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistrar.h"

class UStruct;

class PropertyArgBase;

template <typename TPropertyArgPtr>
concept CPropertyArgPtr =
    std::is_pointer_v<TPropertyArgPtr> &&
    std::derived_from<std::remove_pointer_t<TPropertyArgPtr>, PropertyArgBase>;

template <typename TPropertyArgValueContainer>
concept CPropertyArgValueContainer =
    CValueContainer<TPropertyArgValueContainer> &&
    std::is_pointer_v<typename TPropertyArgValueContainer::value_type> &&
    std::derived_from<std::remove_pointer_t<typename TPropertyArgValueContainer::value_type>, PropertyArgBase>;

template <typename TPropertyArgKeyValueContainer>
concept CPropertyArgKeyValueContainer =
    CKeyValueContainer<TPropertyArgKeyValueContainer> &&
    std::same_as<typename TPropertyArgKeyValueContainer::key_type, std::string> &&
    std::is_pointer_v<typename TPropertyArgKeyValueContainer::mapped_type> &&
    std::derived_from<std::remove_pointer_t<typename TPropertyArgKeyValueContainer::mapped_type>, PropertyArgBase>;

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

    template <CPropertyArgPtr TPropertyArgPtr>
    static void setArgs(std::initializer_list<TPropertyArgPtr> property_args, const std::map<std::string, std::string>& property_strings)
    {
        // convert from initializer_list to vector but don't convert the underlying value type
        std::vector<TPropertyArgPtr> property_args_vector = Std::toVector<TPropertyArgPtr>(property_args);
        setArgs(property_args_vector, property_strings);
    }

    template <CPropertyArgValueContainer TPropertyArgValueContainer>
    static void setArgs(const TPropertyArgValueContainer& property_args, const std::map<std::string, std::string>& property_strings)
    {
        // convert PropertyArg<T>* to PropertyArgBase*
        SP_ASSERT(!Std::contains(property_args, nullptr));
        auto property_arg_map = Std::toMap<std::string, PropertyArgBase*>(
            property_args | std::views::transform([](auto property_arg) { return std::make_pair(property_arg->getName(), property_arg); }));
        setArgs(property_arg_map, property_strings);
    }

    template <CPropertyArgKeyValueContainer TPropertyArgKeyValueContainer>
    static void setArgs(const TPropertyArgKeyValueContainer& property_args, const std::map<std::string, std::string>& property_strings)
    {
        // iterate over property strings, set corresponding property according to string
        for (auto& [property_name, property_string] : property_strings) {
            SP_ASSERT(Std::containsKey(property_args, property_name));
            PropertyArgBase* property_arg = property_args.at(property_name);
            SP_ASSERT(property_arg);
            Unreal::setObjectPropertiesFromString(property_arg->getValuePtr(), property_arg->getStaticStruct(), property_string);
        }
    }
};
