//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function
#include <map>
#include <memory>     // std::make_unique, std::shared_ptr
#include <string>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"

class ClassRegistrationUtils;

// The ClassRegistrationUtils interface is intended to make it possible to instantiate a derived object, given the name
// of a derived class as a string. This is useful when the specific type we want to instantiate is defined in a config
// file. Our first step is to define a static ClassRegistrar object in a location that is visible to all of our possible
// derived types, as well as the code that will be instantiating our derived object. For example, our Agent base class
// defines the following static member variable.
// 
//     inline static auto s_class_registrar_ =
//         ClassRegistrationUtils::getClassRegistrar<Agent, UWorld*>();
//
// The template parameters used here are the base class (Agent), and the types of any arguments (UWorld*) that should be
// passed into the constructor of the derived type. All derived types that we want to instantiate via this ClassRegistrar
// object must provide a public constructor that accepts these types. To register a derived type with this ClassRegistrar
// object, the derived type must define a static ClassRegistrationHandle member variable as follows.
// 
//     inline static auto s_class_registration_handle_ =
//         ClassRegistrationUtils::registerClass<SphereAgent>(Agent::s_class_registrar_, "SphereAgent");
// 
// The template parameters used here are the derived class (SphereAgent). The function arguments are the ClassRegistrar
// object that we want to register with (Agent::s_class_registrar), and name we want to use for the derived type ("SphereAgent").
// 
// Once these static member variables have been defined, we can instantiate a SphereAgent object at runtime as follows.
//
//     Agent* agent =
//         ClassRegistrationUtils::create(Agent::s_class_registrar_, "SphereAgent", world);
//
// The world argument here is a UWorld* pointer. This function will return a new SphereAgent object that was constructed
// using the SphereAgent::SphereAgent(UWorld* world) constructor.

template <typename TBase, typename... TArgs>
class ClassRegistrar
{
    friend class ClassRegistrationUtils;

public:
    template <typename TDerived>
    class ClassRegistrationHandle
    {
    public:
        ClassRegistrationHandle() = delete;

        // We want to modify the original shared_ptr object here, so we pass class_registrar by reference. Roughly speaking, this
        // is the modern C++ equivalent of passing a void** pointer into a function, because the outer code has a pointer that it
        // wants the inner code to modify.
        ClassRegistrationHandle(std::shared_ptr<ClassRegistrar<TBase, TArgs...>>& class_registrar, const std::string& class_name)
        {
            // We usually avoid this type of lazy initialization. But we make an exception in this case because the construction
            // order of a static ClassRegistrar object, relative to the static ClassRegistrationHandle objects that need it, is not
            // guaranteed. So the ClassRegistrar object must be constructed by the first ClassRegistrationHandle object that needs
            // it.
            if (!class_registrar) {
                class_registrar = std::make_shared<ClassRegistrar<TBase, TArgs...>>();
            }

            // Copy class_registrar into an internal shared_ptr, so it will be deleted when the last ClassRegistrationHandle that
            // needs it is deleted.
            class_registrar_ = class_registrar;

            // Add a lambda that creates an instance of the derived type to a map of string-lambda pairs, so we can call it later
            // based on the string.
            SP_ASSERT(!Std::containsKey(class_registrar_->create_funcs_, class_name));
            Std::insert(class_registrar_->create_funcs_, class_name, [](TArgs... args) -> TBase* {
                return new TDerived(args...);
            });
        }

        ~ClassRegistrationHandle()
        {
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
        // objects is not guaranteed. Therefore, a ClassRegistrar must be constructed by the first ClassRegistrationHandle that
        // needs it, not inline in the header where it is declared. So this function just returns a nullptr. Arguably, this
        // doesn't do anything useful at runtime, but we leave it here anyway to make the ClassRegistrationUtils interface
        // slightly more readable and consistent, and to insulate user code from this implementation detail. In other words, we
        // think that this...
        // 
        //     inline static auto s_class_registrar = ClassRegistrationUtils::getClassRegistrar<Agent, UWorld*>();
        // 
        // ...is slightly cleaner than this...
        //
        //     inline static std::shared_ptr<ClassRegistrar<Agent, UWorld*>> s_class_registrar = nullptr;
        // 
        // ...even though both are functionally equivalent.

        return nullptr;
    }

    // We want to modify the original shared_ptr object here, so we pass class_registrar by reference. Roughly speaking, this
    // is the modern C++ equivalent of passing a void** pointer into a function, because the outer code has a pointer that it
    // wants the inner code to modify.
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
