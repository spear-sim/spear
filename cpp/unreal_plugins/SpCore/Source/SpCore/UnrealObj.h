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

class UnrealObjBase;

template <typename TUnrealObjPtr>
concept CUnrealObjPtr =
    std::is_pointer_v<TUnrealObjPtr> &&
    std::derived_from<std::remove_pointer_t<TUnrealObjPtr>, UnrealObjBase>;

template <typename TUnrealObjValueContainer>
concept CUnrealObjValueContainer =
    CValueContainer<TUnrealObjValueContainer> &&
    std::is_pointer_v<typename TUnrealObjValueContainer::value_type> &&
    std::derived_from<std::remove_pointer_t<typename TUnrealObjValueContainer::value_type>, UnrealObjBase>;

template <typename TUnrealObjKeyValueContainer>
concept CUnrealObjKeyValueContainer =
    CKeyValueContainer<TUnrealObjKeyValueContainer> &&
    std::same_as<typename TUnrealObjKeyValueContainer::key_type, std::string> &&
    std::is_pointer_v<typename TUnrealObjKeyValueContainer::mapped_type> &&
    std::derived_from<std::remove_pointer_t<typename TUnrealObjKeyValueContainer::mapped_type>, UnrealObjBase>;

class UnrealObjBase
{
public:
    UnrealObjBase() {};                                       // use when storing in an std::map
    UnrealObjBase(const std::string& name) { name_ = name; }; // use when storing in an std::vector
    virtual ~UnrealObjBase() {};

    virtual void* getValuePtr() = 0;
    virtual UStruct* getStaticStruct() const = 0;

    std::string getName() const { SP_ASSERT(name_ != ""); return name_; };

private:
    std::string name_;
};

template <typename T>
class UnrealObj : public UnrealObjBase
{
public:
    UnrealObj() {};                                              // use when storing in an std::map
    UnrealObj(const std::string& name) : UnrealObjBase(name) {}; // use when storing in an std::vector

    void* getValuePtr() override { return &value; }
    UStruct* getStaticStruct() const override { return UnrealClassRegistrar::getStaticStruct<T>(); }

    UnrealObjBase* getPtr() { return static_cast<UnrealObjBase*>(this); }
    T& getObj() { return value; }

private:
    T value;
};

class UnrealObjUtils
{
public:
    UnrealObjUtils() = delete;
    ~UnrealObjUtils() = delete;

    template <CUnrealObjPtr TUnrealObjPtr>
    static std::map<std::string, std::string> getObjectPropertiesAsStrings(std::initializer_list<TUnrealObjPtr> unreal_objs)
    {
        // convert from initializer_list to vector but don't convert the underlying value type
        return getObjectPropertiesFromStrings(Std::toVector<TUnrealObjPtr>(unreal_objs));
    }

    template <CUnrealObjValueContainer TUnrealObjValueContainer>
    static std::map<std::string, std::string> getObjectPropertiesFromStrings(const TUnrealObjValueContainer& unreal_objs)
    {
        // convert from vector to map and from UnrealObj<T>* to UnrealObjBase*
        SP_ASSERT(!Std::contains(unreal_objs, nullptr));
        auto unreal_obj_map = Std::toMap<std::string, UnrealObjBase*>(
            unreal_objs | std::views::transform([](auto unreal_obj) { return std::make_pair(unreal_obj->getName(), unreal_obj); }));
        return getObjectPropertiesFromStrings(unreal_obj_map);
    }

    template <CUnrealObjKeyValueContainer TUnrealObjKeyValueContainer>
    static std::map<std::string, std::string> getObjectPropertiesFromStrings(const TUnrealObjKeyValueContainer& unreal_objs)
    {
        // input is a map from names to pointers, output is a map from names to property strings
        return Std::toMap<std::string, std::string>(
            unreal_objs |
            std::views::transform([](auto& pair) {
                auto& [name, obj] = pair;
                SP_ASSERT(obj);
                return std::make_pair(name, Unreal::getObjectPropertiesAsString(obj->getValuePtr(), obj->getStaticStruct()));
            }));
    }

    template <CUnrealObjPtr TUnrealObjPtr>
    static void setObjectPropertiesFromStrings(std::initializer_list<TUnrealObjPtr> unreal_objs, const std::map<std::string, std::string>& strings)
    {
        // convert from initializer_list to vector but don't convert the underlying value type
        setObjectPropertiesFromStrings(Std::toVector<TUnrealObjPtr>(unreal_objs), strings);
    }

    template <CUnrealObjValueContainer TUnrealObjValueContainer>
    static void setObjectPropertiesFromStrings(const TUnrealObjValueContainer& unreal_objs, const std::map<std::string, std::string>& strings)
    {
        // convert from vector to map and from UnrealObj<T>* to UnrealObjBase*
        SP_ASSERT(!Std::contains(unreal_objs, nullptr));
        auto unreal_obj_map = Std::toMap<std::string, UnrealObjBase*>(
            unreal_objs | std::views::transform([](auto unreal_obj) { return std::make_pair(unreal_obj->getName(), unreal_obj); }));
        setObjectPropertiesFromStrings(unreal_obj_map, strings);
    }

    template <CUnrealObjKeyValueContainer TUnrealObjKeyValueContainer>
    static void setObjectPropertiesFromStrings(const TUnrealObjKeyValueContainer& unreal_objs, const std::map<std::string, std::string>& strings)
    {
        // iterate over property strings, set corresponding property according to string
        for (auto& [string_name, string] : strings) {
            UnrealObjBase* unreal_obj = unreal_objs.at(string_name);
            SP_ASSERT(unreal_obj);
            Unreal::setObjectPropertiesFromString(unreal_obj->getValuePtr(), unreal_obj->getStaticStruct(), string);
        }
    }
};
