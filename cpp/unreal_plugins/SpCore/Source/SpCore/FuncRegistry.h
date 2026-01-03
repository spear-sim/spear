//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function
#include <string>
#include <map>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"

//
// A FuncRegistry<TReturn, TArgs...> is a templated type that allows a caller to call native C++ functions
// by name. It is especially useful in situations where a user wants to dispatch to a templated function
// based on a string that is only available at runtime.
// 
// The user is responsible for registering each function that may be called through a particular FuncRegistry
// object. For example, a typical user-specified function might call the new operator on a derived type and
// return a base class pointer.
// 
// All functions that are registered with a registry must have the same signature, taking as input TArgs...
// and returning as output TReturn.
//
// The following example demonstrates a typical use case.
// 
//     FuncRegistry<void*, int> new_registry;
//     FuncRegistry<void, void*> delete_registry;
//
// In this case, all functions registered with new_registry must take as input an int and return as output
// void*. Likewise, all functions registered with delete_registry must take as input a void* and return
// void. We can register a specific name and function to our registries by calling registerFunc(...) as
// follows.
//
//     new_registry.registerFunc("float", [](int num_elements) -> void* { return new float[num_elements]; });
//     delete_registry.registerFunc("float", [](void* array) -> void { delete[] static_cast<float*>(array); });
//
// Here, we are registering the name "float" with a create function that allocates an array of floats, and
// a corresponding destroy function that deletes the array. In our destroy function, we need to explicitly
// cast the void* pointer to float* because deleting a void* results in undefined behavior. After we have
// registered our functions, we can call our create and destroy functions using only the type's registered
// name.
//
//     void* my_ptr = new_registry.call("float", 10); // create an array of 10 floats
//     delete_registrar.call("float", my_ptr);        // destroy the array
//
// If we registered other names like "int" and "double" similarly, then we would be able to allocate and
// de-allocate arrays of different types, which would typically require calling a templated function, based
// on a string that is only available at runtime.
//

template <typename TReturn, typename... TArgs>
class FuncRegistry
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

    std::vector<std::string> getFuncNames() const
    {
        return Std::keys(funcs_);
    }

    TReturn call(const std::string& name, TArgs... args) const
    {
        SP_ASSERT(Std::containsKey(funcs_, name));
        return funcs_.at(name)(args...);
    }

private:
    std::map<std::string, std::function<TReturn(TArgs...)>> funcs_;
};
