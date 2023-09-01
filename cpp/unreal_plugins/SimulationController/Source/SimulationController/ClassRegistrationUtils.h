//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"

class ClassRegistrationUtils;

// The ClassRegistrationUtils interface is intended to make it possible to instantiate a derived object, given the name
// of a derived class as a string. This is useful when the specific type we want to instantiate is defined in a config
// file. To accomplish this design goal, our first step is to define a static ClassRegistrar object in a location that
// is visible to all of our possible derived types, as well as the code that will be instantiating our derived object.
// For example, our Agent base class defines the following static member variable.
// 
//     inline static auto s_class_registrar =
//         ClassRegistrationUtils::getClassRegistrar<Agent, UWorld*>();
//
// The template parameters used here are the base class (Agent), and the types of any arguments (UWorld*) that should be
// passed into the constructor of the derived type. All derived types that we want to instantiate via this ClassRegistrar
// object must provide a public constructor that accepts these arguments. To register a derived type with this
// ClassRegistrar object, the derived type must define a static ClassRegistrationHandle member variable as follows.
// 
//     inline static auto s_class_registration_handle =
//         ClassRegistrationUtils::registerClass<SphereAgent>(Agent::s_class_registrar, "SphereAgent");
// 
// The template parameters used here are the derived class (SphereAgent). The function arguments are the ClassRegistrar
// object that we want to register with (Agent::s_class_registrar), and name we want to use for the derived type ("SphereAgent").
// 
// Once these static member variables have been defined, we can instantiate a SphereAgent object as follows.
//
//     Agent* agent = ClassRegistrationUtils::create(Agent::s_class_registrar, "SphereAgent", world);
//
// The world argument here is a UWorld* pointer. This function will return a new SphereAgent object that was constructed
// using the SphereAgent::SphereAgent(UWorld* world) constructor.

template <typename TBase, typename... TArgs>
class ClassRegistrar
{
    friend class ClassRegistrationUtils;

public:
    ClassRegistrar()
    {
        // Use a custom print statement because all the template parameters here confuse our SP_LOG_CURRENT_FUNCTION() macro,
        // and because this code will execute before main(), so can be helpful to see it executing in our debug output.
        SP_LOG("ClassRegistrar::ClassRegistrar()");
    }

    ~ClassRegistrar()
    {
        // Use a custom print statement because all the template parameters here confuse our SP_LOG_CURRENT_FUNCTION() macro,
        // and because this code will execute before main(), so can be helpful to see it executing in our debug output.
        SP_LOG("ClassRegistrar::~ClassRegistrar()");

        create_funcs_.clear();
    }

    template <typename TDerived>
    class ClassRegistrationHandle
    {
    public:
        ClassRegistrationHandle() = delete;

        // We want to modify the original shared_ptr object here, so we pass class_registrar by reference.
        ClassRegistrationHandle(std::shared_ptr<ClassRegistrar<TBase, TArgs...>>& class_registrar, const std::string& class_name)
        {
            // Use a custom print statement because all the template parameters here confuse our SP_LOG_CURRENT_FUNCTION() macro,
            // and because this code will execute before main(), so can be helpful to see it executing in our debug output.
            SP_LOG("ClassRegistrationHandle::ClassRegistrationHandle(", class_registrar.get(), ", \"", class_name, "\")");

            // Lazy initialize class_registrar because the construction order of static objects is not guaranteed.
            if (!class_registrar) {
                class_registrar = std::make_shared<ClassRegistrar<TBase, TArgs...>>();
            }

            // Store a copy of class_registrar in our own shared_ptr, so we will be able to correctly free the memory when the
            // program finishes executing.
            class_registrar_ = class_registrar;

            // Add a lambda that creates an instance of the derived type to a map of string-lambda pairs so we can call it later.
            SP_ASSERT(!Std::containsKey(class_registrar_->create_funcs_, class_name));
            class_registrar_->create_funcs_[class_name] = [](TArgs... args) -> TBase* {
                return new TDerived(args...);
            };
        }

        ~ClassRegistrationHandle()
        {
            SP_LOG("ClassRegistrationHandle::~ClassRegistrationHandle()");

            class_registrar_ = nullptr;
        }

    private:
        std::shared_ptr<ClassRegistrar<TBase, TArgs...>> class_registrar_ = nullptr;
    };

private:
    std::map<std::string, std::function<TBase*(TArgs...)>> create_funcs_;
};

class ClassRegistrationUtils
{
public:
    template <typename TBase, typename... TArgs>
    static std::shared_ptr<ClassRegistrar<TBase, TArgs...>> getClassRegistrar()
    {
        // A static std::shared_ptr<ClassRegistrar> should be initialized to nullptr, because the construction order of static
        // objects is not guaranteed. So this function doesn't really do anything useful at runtime. It's purpose is to make it
        // possible to use auto at the call site to make it slightly more readable. For example, we think that this...
        // 
        //     inline static auto s_class_registrar = ClassRegistrationUtils::getClassRegistrar<Agent, UWorld*>();
        // 
        // ...is slightly cleaner than this...
        //
        //     inline static std::shared_ptr<ClassRegistrar<Agent, UWorld*>> s_class_registrar = nullptr;
        // 
        // ...but both are functionally equivalent.

        return nullptr;
    }

    // We want to modify the original shared_ptr object here, so we pass class_registrar by reference.
    template <typename TDerived, typename TBase, typename... TArgs>
    static typename ClassRegistrar<TBase, TArgs...>::template ClassRegistrationHandle<TDerived> registerClass(
        std::shared_ptr<ClassRegistrar<TBase, TArgs...>>& class_registrar, const std::string& class_name)
    {
        return typename ClassRegistrar<TBase, TArgs...>::template ClassRegistrationHandle<TDerived>(class_registrar, class_name);
    }

    template <typename TBase, typename... TArgs>
    static TBase* create(std::shared_ptr<ClassRegistrar<TBase, TArgs...>>& class_registrar, const std::string& class_name, TArgs... args)
    {
        SP_ASSERT(class_registrar);
        return class_registrar->create_funcs_.at(class_name)(args...);
    }
};
