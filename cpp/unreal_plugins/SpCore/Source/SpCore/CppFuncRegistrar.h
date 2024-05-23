//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function
#include <string>
#include <map>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

//
// A CppFuncRegistrar<TReturn, TArgs...> is a templated type that allows a caller to call native C++
// functions by name. The user is responsible for registering each function that may be called through a
// particular CppFuncRegistrar object. For example, a typical user-specified function might call the new
// operator on a derived type and return a base class pointer.
// 
// All functions that are registered with a registrar must have the same signature, taking as input TArgs...
// and returning as output TReturn.
//
// This following example demonstrates a typical use case.
// 
//     CppFuncRegistrar<void*, int> new_registrar;
//     CppFuncRegistrar<void, void*> delete_registrar;
//
// In this case, all functions registered with new_registrar must take as input an int and return as output
// void*. Likewise, all functions registered with delete_registrar must take as input a void* and return
// void. We can register a specific name and function to our registrars by calling registerFunc(...) as
// follows.
//
//     new_registrar.registerFunc("float", [](int num_elements) -> void* { return new float[num_elements]; });
//     delete_registrar.registerFunc("float", [](void* array) -> void { delete[] static_cast<float*>(array); });
//
// Here, we are registering the name "float" with a create function that allocates an array of floats, and
// a corresponding destroy function that deletes the array. In our destroy function, we need to explicitly
// cast the void* pointer to float* because deleting a void* results in undefined behavior. After we have
// registered our functions, we can call our create and destroy functions using only the type's registered
// name.
//
//     void* my_ptr = new_registrar.call("float", 10); // create an array of 10 floats
//     delete_registrar.call("float", my_ptr);         // destroy the array
//

template <typename TReturn, typename... TArgs>
class CppFuncRegistrar
{
public:
    void registerFunc(const std::string& name, const std::function<TReturn(TArgs...)>& func)
    {
        SP_ASSERT(func);
        SP_ASSERT(name != "");
        Std::insert(funcs_, name, func);
    }

    void unregisterFunc(const std::string& name)
    { 
        SP_ASSERT(name != "");
        Std::remove(funcs_, name);
    }

    TReturn call(const std::string& name, TArgs... args) const
    {
        SP_ASSERT(Std::containsKey(funcs_, name));
        return funcs_.at(name)(args...);
    }

private:
    std::map<std::string, std::function<TReturn(TArgs...)>> funcs_;
};
